/**
 * @file webmerc.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Web-Mercator GDAL driver implementation
 *
 * Adds to GDAL an ability to work with map data in the web-mercator format
 * Dataset uri: PROVIDER://MAP_TYPE/ZOOM where:
 *    * PROVIDER is one of mapycz, google, ...
 *    * MAP_TYPE selects one of provider's map
 *    * ZOOM is a level-of-detail specifier
 */

#ifndef gdal_drivers_webmerc_hpp_included_
#define gdal_drivers_webmerc_hpp_included_

#include <gdal_priv.h>

#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include "math/geometry_core.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

class WebMercatorRasterBand;

namespace detail {
    class Fetcher;
} // namespace detail

/**
 * @brief GttDataset
 */

class WebMercatorDataset : public GDALDataset {

    friend class WebMercatorRasterBand;

public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~WebMercatorDataset();

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

private:
    WebMercatorDataset(const std::string &urlTemplate
                       , int zoom, unsigned int flags);

    const cv::Mat& getTile(const math::Point2i &tile);

    int zoom_;

    std::string srs_;

    math::Point2i lastTile_;
    cv::Mat lastTileImage_;

    enum Flag : unsigned int { none = 0x0 };

    unsigned int flags_;

    std::unique_ptr<detail::Fetcher> fetcher_;

    std::string urlTemplate_;
};

/**
 * @brief WebMercatorRasterBand
 */

class WebMercatorRasterBand : public GDALRasterBand
{

public:
    WebMercatorRasterBand(WebMercatorDataset *dset, int numBand
                     , int cvChannel
                     , GDALColorInterp colorInterp);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~WebMercatorRasterBand() {};

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
void GDALRegister_WebMercator(void);
CPL_C_END

#endif // gdal_drivers_webmerc_hpp_included_
