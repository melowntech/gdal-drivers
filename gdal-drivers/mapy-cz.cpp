/*
 * mapy-cz.cpp
 */

#include <unistd.h>

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/uri.hpp"
#include "utility/path.hpp"
#include "utility/time.hpp"
#include "geo/srsdef.hpp"
#include "imgproc/readimage.hpp"

#include "./mapy-cz.hpp"
#include "./detail/localcache.hpp"

namespace gdal_drivers {

namespace {

namespace def {
    // media URL
    const std::string MediaUrl("http://m1.mapserver.mapy.cz/%s/%i_%07x_%07x");
    // fake referer to not to look so suspicious in the Seznam.cz log :)
    const std::string RefererUrl("http://mapy.cz/");

    const std::string Schema("mapycz");
    const std::string DefaultMapType("ophoto");

    const math::Size2i TileSize(256, 256);
    const math::Extents2 PPExtents(-3700000.0, 1300000.0
                                   , -3700000.0 + (1 << 23)
                                   , 1300000.0 + (1 << 23));

    std::string Srs("+proj=utm +zone=33 +ellps=WGS84 +datum=WGS84");

    const char *ProxyEnv("MAPYCZ_PROXY");
    const char *CacheEnv("MAPYCZ_CACHEPATH");
} // namespace constants

std::string makeUrl(const std::string mapType, int zoom, long x, long y)
{
    return str(boost::format(def::MediaUrl) % mapType % zoom % x % y);
}

Curl createCurl()
{
    auto c(::curl_easy_init());
    if (!c) {
        LOGTHROW(err2, std::runtime_error) << "Failed to create CURL handle.";
    }

    // switch off SIGALARM
    ::curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1);

    return Curl(c, [](CURL *c) { if (c) { ::curl_easy_cleanup(c); } });
}

} // namespace

/* class MapyczDataset */

GDALDataset* MapyczDataset::Open(GDALOpenInfo *openInfo)
{
    // parse path
    auto uri(utility::parseUri(openInfo->pszFilename));

    // check schema
    if (uri.schema != def::Schema) { return nullptr; }

    // get map type (from uri host, with fallback to default map type
    auto mapType(uri.host.empty() ? def::DefaultMapType : uri.host);

    // parse zoom
    int zoom;
    try {
        auto zc(utility::pathComponent(uri.path, 1));
        if (!zc) { return nullptr; }
        zoom  = boost::lexical_cast<int>(zc->string());
    } catch (const boost::bad_lexical_cast&) {
        return nullptr;
    }
    LOG(debug) << "zoom: " << zoom;

    // modifiers
    unsigned int flags(Flag::none);
    unsigned int index(0);
    for (const auto &c : fs::path(uri.path)) {
        // skip root and zoom
        if (index++ < 2) { continue; }

        (void) c;
        LOG(err2) << "Invalid modifier.";
        return nullptr;
    }

    // no updates
    if (openInfo->eAccess == GA_Update) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "The MapyCz driver does not support update access to existing"
                 " datasets (you know, Seznam.cz would not be happy"
                 " otherwise).\n");
        return 0x0;
    }

    // initialize dataset
    try {
        return new MapyczDataset(mapType, zoom, flags);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

namespace {

std::unique_ptr<detail::LocalCache> createCache()
{
    if (const char *cacheRoot = std::getenv(def::CacheEnv)) {
        return std::unique_ptr<detail::LocalCache>
            (new detail::LocalCache(cacheRoot));
    }

    return {};
}

} // namespace

MapyczDataset::~MapyczDataset() {}

MapyczDataset::MapyczDataset(const std::string &mapType, int zoom
                             , unsigned int flags)
    : mapType_(mapType), zoom_(zoom)
    , srs_(geo::SrsDefinition(def::Srs).as(geo::SrsDefinition::Type::wkt).srs)
    , pixelSize_(std::pow(2.0, 15 - zoom), std::pow(2.0, 15 - zoom))
    , tileSize_((1 << (28 - zoom)), 1 << (28 - zoom))
    , curl_(createCurl()), lastTile_(-1, -1)
    , flags_(flags), cache_(createCache())
{
    // calculate size of raster in pixels
    nRasterXSize = (def::TileSize.width << zoom);
    nRasterYSize = (def::TileSize.height << zoom);

    // full RGB
    int index(1);
    int cvChannel(2);
    for (auto colorInterp : { GCI_RedBand, GCI_GreenBand, GCI_BlueBand }) {
        SetBand(index, new MapyczRasterBand
                    (this, index, cvChannel, colorInterp));
        ++index;
        --cvChannel;
    }

    // set some useful information into metadata
    SetMetadataItem("ZOOM"
                    , boost::lexical_cast<std::string>(zoom_).c_str()
                    , "MAPYCZ");

    SetMetadataItem("TILE_GEO_WIDTH"
                    , boost::lexical_cast<std::string>
                    (1 << (23 - zoom_)).c_str()
                    , "MAPYCZ");

    SetMetadataItem("TILE_GEO_HEIGHT"
                    , boost::lexical_cast<std::string>
                    (1 << (23 - zoom_)).c_str()
                    , "MAPYCZ");
}

CPLErr MapyczDataset::GetGeoTransform(double *padfTransform) {

    padfTransform[0] = def::PPExtents.ll[0];
    padfTransform[1] = pixelSize_.width;
    padfTransform[2] = 0.0;

    padfTransform[3] = def::PPExtents.ur[1];
    padfTransform[4] = 0.0;
    padfTransform[5] = -pixelSize_.height;

    return CE_None;
}

const char* MapyczDataset::GetProjectionRef()
{
    return srs_.c_str();
}

extern "C" {

typedef std::vector<char> Buffer;

size_t gdal_drivers_mapy_cz_write(char *ptr, size_t size, size_t nmemb
                                  , void *userdata)
{
    auto &buffer(*static_cast<Buffer*>(userdata));
    auto bytes(size * nmemb);
    std::copy(ptr, ptr + bytes, std::back_inserter(buffer));
    return bytes;
}

} // extern "C"

namespace {

cv::Mat decodeImage(const std::string &url
                    , const Buffer &buffer, bool skipCorrupted)
{
    auto image(imgproc::readImage(buffer.data(), buffer.size()));
    if (!image.data) {
        if (skipCorrupted) {
            LOG(warn2) << "Failed to decode tile data downloaded from <"
                       << url << ">, using black image instead.";
            image = cv::Mat( def::TileSize.height, def::TileSize.width
                           , CV_8UC3, cv::Scalar(1, 1, 1));
        } else {
            LOGTHROW(err2, std::runtime_error)
                << "Failed to decode tile data downloaded from <"
                << url << ">.";
        }
    }

    if ((image.cols != def::TileSize.width)
        || (image.rows != def::TileSize.height))
    {
        LOGTHROW(err2, std::runtime_error)
            << "Tile downloaded from from <"
            << url << "> has wrong dimensions: "
            << image.cols << "x" << image.rows << " (should be "
            << def::TileSize << ").";
    }

    // convert 0 to 1 to prevent interpretation as no-data value
    {
        auto src(static_cast<unsigned char*>(image.data));
        std::size_t width(image.cols * image.channels());
        for (std::size_t y(0); y < std::size_t(image.rows); ++y) {
            auto *sdata(src + image.step * y);
            for (std::size_t x(0); x < width; ++x, ++sdata) {
                if (!*sdata) { *sdata = 1; }
            }
        }
    }

    return image;
}

long int fetchUrl(::CURL *curl, const std::string &url, Buffer &buffer)
{
    LOG(info1) << "Fetching tile from <" << url << "> "
               << "(GET).";

    buffer.clear();

#define CHECK_CURL_STATUS(what)                                         \
    do {                                                                \
        auto res(what);                                                 \
        if (res != CURLE_OK) {                                          \
            LOGTHROW(err2, std::runtime_error)                          \
                << "Failed to download tile from <"                     \
                << url << ">: <" << res << ", "                         \
                << ::curl_easy_strerror(res)                            \
                << ">.";                                                \
        }                                                               \
    } while (0)

    // we are getting a resource
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L));
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_NOBODY, 0L));

    // HTTP/1.1
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_HTTP_VERSION
                                         , CURL_HTTP_VERSION_1_1));
    // target url
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_URL, url.c_str()));

    // do not follow redirects -> we can detect non-existent tiles
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L));

    // set referer ;)
    CHECK_CURL_STATUS(::curl_easy_setopt
                      (curl, CURLOPT_REFERER, def::RefererUrl.c_str()));

    // use proxy if set in environment
    if (const char *proxy = std::getenv(def::ProxyEnv)) {
        LOG(info1) << "Using proxy server at <" << proxy << ">.";
        CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_PROXY, proxy));
    }

    // set output function + userdata
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION
                                         , &gdal_drivers_mapy_cz_write));
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer));

    /// do the thing
    CHECK_CURL_STATUS(::curl_easy_perform(curl));

    // check status code:
    long int httpCode(0);
    CHECK_CURL_STATUS(::curl_easy_getinfo
                      (curl, CURLINFO_RESPONSE_CODE, &httpCode));
#undef CHECK_CURL_STATUS
    return httpCode;
}

cv::Mat blackTile()
{
    cv::Mat black(def::TileSize.height, def::TileSize.width, CV_8UC(3)
                  , cv::Scalar(0, 0, 0));
    return black;
}

cv::Mat fetchTile(detail::LocalCache *cache
                  , ::CURL *curl, const std::string &url
                  , bool skipCorrupted)
{
    using detail::LocalCache;
    LocalCache::Tile tile;
    if (cache) {
        tile = cache->fetchTile(url);

        switch (tile.type) {
        case LocalCache::Tile::Type::empty:
            return blackTile();

        case LocalCache::Tile::Type::valid:
            // tile loaded from cache
            try {
                auto image(decodeImage(url, tile.data, false));
                LOG(info1) << "Tile from <" << url << "> fetched (cached).";
                return image;
            } catch (std::exception&) {
                // cannot decode
            }

        default: break; // not found
        }
    }

    // fetch from web

    auto httpCode(fetchUrl(curl, url, tile.data));

    if (httpCode == 302) {
        // TODO: check redirect location
        // no imagery for this tile -> return black one
        LOG(info1) << "Tile from <" << url << "> fetched.";

        if (cache) {
            tile.type = LocalCache::Tile::Type::empty;
            tile.expires = utility::currentTime().first + 604800; // a week
            cache->storeTile(url, tile);
        }

        return blackTile();
    }

    if (httpCode != 200) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to download tile data from <"
            << url << ">: Unexpected HTTP status code: <" << httpCode << ">.";
    }

    auto image(decodeImage(url, tile.data, skipCorrupted));

    if (cache) {
        tile.type = LocalCache::Tile::Type::valid;
        tile.expires = utility::currentTime().first + 604800; // a week
        cache->storeTile(url, tile);
    }

    LOG(info1) << "Tile from <" << url << "> fetched.";
    return image;
}

cv::Mat fetchTileSafe(detail::LocalCache *cache
                      , ::CURL *curl, const std::string &url
                      , unsigned int tries)
{
    for (; tries; --tries) {
        try {
            return fetchTile(cache, curl, url, false);
        } catch (const std::exception &e) {
            LOG(warn2) << "Failed to fetch tile, retrying.";
            ::sleep(1);
        }
    }

    // final try, let's it fall through on failure
    return fetchTile(cache, curl, url, true);
}

} // namespace

const cv::Mat& MapyczDataset::getTile(const math::Point2i &tile)
{
    if (lastTileImage_.data && (tile == lastTile_)) {
        // tile cached
        return lastTileImage_;
    }

    auto url(makeUrl(mapType_, zoom_, tile(0) * tileSize_.width
                     , (1 << 28) - ((tile(1) + 1) * tileSize_.height)));

    auto image(fetchTileSafe(cache_.get(), curl_.get(), url, 20));

    // remember
    lastTileImage_ = image;
    lastTile_ = tile;

    // done
    return lastTileImage_;
}

/* MapyczRasterBand */

MapyczRasterBand::MapyczRasterBand(MapyczDataset *dset, int numBand
                                   , int cvChannel
                                   , GDALColorInterp colorInterp)
    : cvChannel_(cvChannel), colorInterp_(colorInterp)
{
    poDS = dset;
    nBand = numBand;
    nBlockXSize = def::TileSize.width;
    nBlockYSize = def::TileSize.height;
    eDataType = GDT_Byte;
}

CPLErr MapyczRasterBand::IReadBlock(int blockCol, int blockRow
                                    , void *rawImage)
{
    auto &dset(*static_cast<MapyczDataset*>(poDS));

    try {
        cv::Mat tile[1] = { dset.getTile({blockCol, blockRow}) };
        cv::Mat image[1]
        {{def::TileSize.height, def::TileSize.width, CV_8UC1, rawImage}};

        int fromTo[2] = { cvChannel_, 0 };
        cv::mixChannels({ tile }, 1, image, 1, fromTo, 1);
    } catch (const std::exception &e) {
        CPLError(CE_Failure, CPLE_FileIO, "%s\n", e.what());
        return CE_Failure;
    }
    return CE_None;
}

} // namespace gdal_drivers

/* GDALRegister_Mapycz */

void GDALRegister_MapyCz()
{
    if (!GDALGetDriverByName("mapy.cz")) {
        std::unique_ptr<GDALDriver> driver(new GDALDriver());

        driver->SetDescription("mapy.cz");
        driver->SetMetadataItem(GDAL_DMD_LONGNAME
                                , "Mapy.cz map layer support.");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        driver->pfnOpen = gdal_drivers::MapyczDataset::Open;

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
