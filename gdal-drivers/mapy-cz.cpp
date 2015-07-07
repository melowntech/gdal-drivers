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
#include <boost/algorithm/string/predicate.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/uri.hpp"
#include "utility/path.hpp"
#include "geo/srsdef.hpp"

#include "./mapy-cz.hpp"
#include "./detail/fetcher.hpp"

namespace gdal_drivers {

namespace ba = boost::algorithm;

namespace {

namespace def {
    // media URL
    const std::string MediaUrl("http://m1.mapserver.mapy.cz/%s/%i_%07x_%07x");
    // fake referer to not to look so suspicious in the Seznam.cz log :)
    const std::string RefererUrl("http://mapy.cz/");

    const std::string Schema("mapycz");

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

template <typename T>
boost::optional<T> getOptionalEnv(const char *var)
{
    if (const char *value = std::getenv(var)) {
        return T(value);
    }
    return boost::none;
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
    auto mapType(uri.host);

    if (ba::ends_with(mapType, "-m")) {
        // mercator -> let the Webmercator module handle this
        return nullptr;
    }

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

MapyczDataset::~MapyczDataset() {}

MapyczDataset::MapyczDataset(const std::string &mapType, int zoom
                             , unsigned int flags)
    : mapType_(mapType), zoom_(zoom)
    , srs_(geo::SrsDefinition(def::Srs).as(geo::SrsDefinition::Type::wkt).srs)
    , pixelSize_(std::pow(2.0, 15 - zoom), std::pow(2.0, 15 - zoom))
    , tileSize_((1 << (28 - zoom)), 1 << (28 - zoom))
    , lastTile_(-1, -1), flags_(flags)
    , fetcher_(new detail::Fetcher
               (def::TileSize
                , getOptionalEnv<std::string>(def::ProxyEnv)
                , getOptionalEnv<fs::path>(def::CacheEnv)))
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

const cv::Mat& MapyczDataset::getTile(const math::Point2i &tile)
{
    if (lastTileImage_.data && (tile == lastTile_)) {
        // tile cached
        return lastTileImage_;
    }

    auto url(makeUrl(mapType_, zoom_, tile(0) * tileSize_.width
                     , (1 << 28) - ((tile(1) + 1) * tileSize_.height)));

    auto image(fetcher_->getTile(url));

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
