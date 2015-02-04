/**
 * @file mapy-cz.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Bordered Aread GDAL driver implementation
 *
 * Adds to GDAL an ability to work with special mask data set:
 *     * mask is stored in GeoTiff
 *     * each pixel represents 256x256 tile
 *     * tile inside border is full white
 *     * tile outside border is full black
 *     * tile exactly on border is stored in its one PNG image
 */

#ifndef gdal_drivers_borderedarea_hpp_included_
#define gdal_drivers_borderedarea_hpp_included_

#include <gdal_priv.h>

#include <map>

#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include <curl/curl.h>

#include "math/geometry_core.hpp"

#include "geo/geodataset.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

class BorderedAreaRasterBand;

/**
 * @brief GttDataset
 */

class BorderedAreaDataset : public GDALDataset {
public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~BorderedAreaDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

    friend class BorderedAreaRasterBand;

private:
    BorderedAreaDataset(const fs::path &root);

    const cv::Mat& getTile(const math::Point2i &tile);

    const fs::path root_;
    geo::GeoDataset mask_;
    std::string srs_;
    math::Size2 tileSize_;

    // List tile read
    math::Point2i lastTile_;
    cv::Mat lastTileImage_;

    // last block read from underlying GeoTiff
    geo::GeoDataset::Block lastBlock_;
    math::Point2i lastBlockOffset_;

    /** Tile full of black pixels
     */
    cv::Mat blackTile_;

    /** Tile full of white pixels
     */
    cv::Mat whiteTile_;
};

/**
 * @brief BorderedAreaRasterBand
 */

class BorderedAreaRasterBand : public GDALRasterBand
{

public:
    BorderedAreaRasterBand(BorderedAreaDataset *dset);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~BorderedAreaRasterBand() {};

    /** 0 is special marker for no-data pixels
     */
    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 1; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return GCI_AlphaBand; }

private:
    int cvChannel_;
    GDALColorInterp colorInterp_;
};

} // namespace gdal_drivers


// driver registration function
CPL_C_START
void GDALRegister_BorderedArea(void);
CPL_C_END

#endif // gdal_drivers_borderedarea_hpp_included_
