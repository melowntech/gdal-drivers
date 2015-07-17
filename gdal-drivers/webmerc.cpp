/*
 * webmerc.cpp
 */

#include <unistd.h>

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/uri.hpp"
#include "utility/path.hpp"
#include "geo/srsdef.hpp"

#include "./webmerc.hpp"
#include "./detail/fetcher.hpp"

namespace gdal_drivers {

namespace ba = boost::algorithm;

namespace {

namespace def {
    const std::map<std::string, std::string> UrlMap = {
        { "GoogleSat", "https://khms${GOOG_DIGIT}.google.com/kh/v=175&x=${X}&y=${Y}&z=${ZOOM}" }
        , { "GoogleMap", "https://mt${GOOG_DIGIT}.google.com/vt/lyrs=m@132&hl=pt-PT&x=${X}&y=${Y}&z=${ZOOM}&s=${GALILEO}" }

        , { "MicrosoftHyb", "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/h${QUAD}.png?g=441&mkt=en-us&n=z" }
        , { "MicrosoftSat", "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/a${QUAD}.png?g=441&mkt=en-us&n=z" }
        , { "MicrosoftMap", "http://ecn.t${MS_DIGIT}.tiles.virtualearth.net/tiles/r${QUAD}.png?g=441&mkt=en-us&n=z" }

        , { "OviSat", "http://maptile.maps.svc.ovi.com/maptiler/v2/maptile/newest/satellite.day/${ZOOM}/${X}/${Y}/256/png8" }
        , { "OviHybrid", "http://maptile.maps.svc.ovi.com/maptiler/v2/maptile/newest/hybrid.day/${ZOOM}/${X}/${Y}/256/png8" }

        , { "OpenStreetMap", "http://tile.openstreetmap.org/${ZOOM}/${X}/${Y}.png" }

        , { "MapyCzOphoto", "http://m1.mapserver.mapy.cz/ophoto-m/${ZOOM}-${X}-${Y}" }
        , { "MapyCzBase", "http://m1.mapserver.mapy.cz/base-m/${ZOOM}-${X}-${Y}" }
        , { "MapyCzArmy2", "http://m1.mapserver.mapy.cz/army2-m/${ZOOM}-${X}-${Y}" }

        , { "ArcGISTopo", "http://services.arcgisonline.com/ArcGIS/rest/services/World_Topo_Map/MapServer/tile/${ZOOM}/${Y}/${X}.png" }
    };

    const std::map<std::string, std::string> SupportedSources = {
        { "google/", "GoogleSat" }
        , { "google/satellite", "GoogleSat" }
        , { "google/map", "GoogleMap" }

        , { "bing/", "MicrosoftSat" }
        , { "bing/hybride", "MicrosoftHyb" }
        , { "bing/satellite", "MicrosoftSat" }
        , { "bing/map", "MicrosoftMap" }

        , { "ovi/", "OviSat" }
        , { "ovi/satellite", "OviSat" }
        , { "ovi/hybrid", "OviHybrid" }

        , { "osm/", "OpenStreetMap" }
        , { "osm/map", "OpenStreetMap" }

        , { "mapycz/", "MapyCzOphoto" }
        , { "mapycz/ophoto-m", "MapyCzOphoto" }
        , { "mapycz/base-m", "MapyCzBase" }
        , { "mapycz/army2-m", "MapyCzArmy2" }

        , { "arcgis/", "ArcGISTopo" }
        , { "arcgis/topo", "ArcGISTopo" }
    };

    const math::Size2i TileSize(256, 256);

    const math::Extents2 WMExtents(-20037508.342789, -20037508.342789
                                   , 20037508.342789, 20037508.342789);

    const auto WMSize(math::size(WMExtents));

    const char *ProxyEnv("GDAL_HTTPPROXY");
    const char *CacheEnv("GDAL_CACHEPATH");
} // namespace constants

std::string buildQuad(int x, int y, int zoom)
{
    std::string q;

    for (int i = zoom - 1; i >=0 ; --i) {
        q.append(str(boost::format("%d")
                     % (((((y >> i) & 1) << 1) + ((x >> i) & 1)))));
    }

    return q;
}

struct TileServiceInfo {
    TileServiceInfo(const std::string &url, int x, int y, int zoom)
        : x(x)
        , y(y)
        , zoom(zoom)
        , quad(buildQuad(x, y, zoom))
        , oamZoom(17 - zoom)
        , googDigit((x + y) & 3)
        , msDigirBr((((y & 1) << 1) + (x & 1)) + 1)
        , msDigit(((y & 3) << 1) + (x & 1))
        , yDigit((x + y + zoom) % 3 + 1)
        , galileo(std::string("Galileo").substr(0, (3 * x + y) & 7))
        , url(formatUrl(url))
    {}

    static std::string translateTemplate(std::string temp);

    int x;
    int y;
    int zoom;
    std::string quad;
    int oamZoom;
    int googDigit;
    int msDigirBr;
    int msDigit;
    int yDigit;
    std::string galileo;

    std::string url;

private:
    std::string formatUrl(const std::string &u) {
        boost::format format(u);
        format.exceptions(boost::io::all_error_bits
                          ^ boost::io::too_many_args_bit);
        return str(format
                   % x
                   % y
                   % zoom
                   % quad
                   % oamZoom
                   % googDigit
                   % msDigirBr
                   % msDigit
                   % yDigit
                   % galileo);
    }
};

std::string TileServiceInfo::translateTemplate(std::string temp)
{
    int index(1);
    for (const char *name : {
            "X"
            , "Y"
            , "ZOOM"
            , "QUAD"
            , "OAM_ZOOM"
            , "GOOG_DIGIT"
            , "MS_DIGITBR"
            , "MS_DIGIT"
            , "Y_DIGIT"
            , "GALILEO"
        })
    {
        ba::replace_all(temp, str(boost::format("${%1%}") % name)
                        , str(boost::format("%%%1%%%") % index));
        ++index;
    }

    return temp;
}

std::string makeUrl(const std::string urlTemplate, int zoom, long x, long y)
{
    return TileServiceInfo(urlTemplate, x, y, zoom).url;
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

/* class WebMercatorDataset */

GDALDataset* WebMercatorDataset::Open(GDALOpenInfo *openInfo)
{
    // parse path
    auto uri(utility::parseUri(openInfo->pszFilename));

    try {
        boost::lexical_cast<int>(uri.host);
        // host is number -> update uri
        uri.path = "/" + uri.host + uri.path;
        uri.host = "";
    } catch (boost::bad_lexical_cast) {}

    auto fSupportedSources
        (def::SupportedSources.find((uri.schema + "/" + uri.host)));
    if (fSupportedSources == def::SupportedSources.end()) {
        // schema/host combination not found
        return nullptr;
    }

    auto fUrlMap(def::UrlMap.find(fSupportedSources->second));
    if (fUrlMap == def::UrlMap.end()) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "WebMercator: internal error: no url definition for %s.\n"
                 , fSupportedSources->second.c_str());
        return nullptr;
    }

    const std::string urlTemplate
        (TileServiceInfo::translateTemplate(fUrlMap->second));

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
                 "The WebMercator driver does not support update "
                 "access to existing datasets.\n");
        return 0x0;
    }

    // initialize dataset
    try {
        return new WebMercatorDataset(urlTemplate, zoom, flags);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

WebMercatorDataset::~WebMercatorDataset() {}

WebMercatorDataset::WebMercatorDataset(const std::string &urlTemplate, int zoom
                                       , unsigned int flags)
    : zoom_(zoom)
    , srs_(geo::SrsDefinition("3857", geo::SrsDefinition::Type::epsg)
           .as(geo::SrsDefinition::Type::wkt).srs)
    , lastTile_(-1, -1), flags_(flags)
    , fetcher_(new detail::Fetcher
               (def::TileSize
                , getOptionalEnv<std::string>(def::ProxyEnv)
                , getOptionalEnv<fs::path>(def::CacheEnv)))
    , urlTemplate_(urlTemplate)
{
    // calculate size of raster in pixels
    nRasterXSize = (def::TileSize.width << zoom);
    nRasterYSize = (def::TileSize.height << zoom);

    // full RGB
    int index(1);
    int cvChannel(2);
    for (auto colorInterp : { GCI_RedBand, GCI_GreenBand, GCI_BlueBand }) {
        SetBand(index, new WebMercatorRasterBand
                    (this, index, cvChannel, colorInterp));
        ++index;
        --cvChannel;
    }

    // set some useful information into metadata
    SetMetadataItem("ZOOM"
                    , boost::lexical_cast<std::string>(zoom_).c_str()
                    , "WEBMERCATOR");

    SetMetadataItem("TILE_GEO_WIDTH"
                    , boost::lexical_cast<std::string>
                    (1 << (23 - zoom_)).c_str()
                    , "WEBMERCATOR");

    SetMetadataItem("TILE_GEO_HEIGHT"
                    , boost::lexical_cast<std::string>
                    (1 << (23 - zoom_)).c_str()
                    , "WEBMERCATOR");
}

CPLErr WebMercatorDataset::GetGeoTransform(double *padfTransform)
{
    padfTransform[0] = def::WMExtents.ll[0];
    padfTransform[1] = def::WMSize.width / nRasterXSize;
    padfTransform[2] = 0.0;

    padfTransform[3] = def::WMExtents.ur[1];
    padfTransform[4] = 0.0;
    padfTransform[5] = -def::WMSize.height / nRasterYSize;

    return CE_None;
}

const char* WebMercatorDataset::GetProjectionRef()
{
    return srs_.c_str();
}

const cv::Mat& WebMercatorDataset::getTile(const math::Point2i &tile)
{
    if (lastTileImage_.data && (tile == lastTile_)) {
        // tile cached
        return lastTileImage_;
    }

    auto url(makeUrl(urlTemplate_, zoom_, tile(0), tile(1)));

    auto image(fetcher_->getTile(url));

    // remember
    lastTileImage_ = image;
    lastTile_ = tile;

    // done
    return lastTileImage_;
}

/* WebMercatorRasterBand */

WebMercatorRasterBand::WebMercatorRasterBand(WebMercatorDataset *dset, int numBand
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

CPLErr WebMercatorRasterBand::IReadBlock(int blockCol, int blockRow
                                    , void *rawImage)
{
    auto &dset(*static_cast<WebMercatorDataset*>(poDS));

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

/* GDALRegister_WebMercator */

void GDALRegister_WebMercator()
{
    if (!GDALGetDriverByName("webmercator")) {
        std::unique_ptr<GDALDriver> driver(new GDALDriver());

        driver->SetDescription("webmercator");
        driver->SetMetadataItem(GDAL_DMD_LONGNAME
                                , "WebMercator map layer support.");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        driver->pfnOpen = gdal_drivers::WebMercatorDataset::Open;

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
