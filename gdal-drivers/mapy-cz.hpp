/**
 * @file mapy-cz.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Mapy.cz GDAL driver implementation
 *
 * Adds to GDAL an ability to work with map data from mapy.cz.
 * Dataset uri: mapycz://MAP_TYPE/ZOOM where:
 *    * MAP_TYPE is one of base, ortho
 *    * ZOOM is a level-of-detail specifier in range 0-20
 */

#ifndef gdal_drivers_mapy_cz_hpp_included_
#define gdal_drivers_mapy_cz_hpp_included_

#include <gdal_priv.h>

#include <map>

#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include <curl/curl.h>

#include "math/geometry_core.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

typedef std::shared_ptr< ::CURL> Curl;

class MapyczRasterBand;

/**
 * @brief GttDataset
 */

class MapyczDataset : public GDALDataset {

    friend class MapyczRasterBand;

public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~MapyczDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

private:
    MapyczDataset(const std::string &mapType, int zoom);

    const cv::Mat& getTile(const math::Point2i &tile);

    const std::string mapType_;
    int zoom_;

    std::string srs_;

    /** Pixel size in meters.
     */
    math::Size2f pixelSize_;

    /** Tile size in PP units.
     */
    math::Size2_<long> tileSize_;

    Curl curl_;

    math::Point2i lastTile_;
    cv::Mat lastTileImage_;
};

/**
 * @brief MapyczRasterBand
 */

class MapyczRasterBand : public GDALRasterBand
{

public:
    MapyczRasterBand(MapyczDataset *dset, int numBand
                     , int cvChannel
                     , GDALColorInterp colorInterp);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~MapyczRasterBand() {};

    /** 0 is special marker for no-data pixels
     * NB: all valid pixels with 0 in original data are changed to 1 internally
     */
    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 1; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return colorInterp_; }

private:
    int cvChannel_;
    GDALColorInterp colorInterp_;
};

} // namespace gdal_drivers


// driver registration function
CPL_C_START
void GDALRegister_MapyCz(void);
CPL_C_END

#endif // gdal_drivers_mapy_cz_hpp_included_
