/**
 * @file mask.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Quadtree Mask GDAL driver implementation
 *
 * Adds to GDAL an ability to work with mask dataset defined as a quadtree.
 */

#ifndef gdal_drivers_mask_hpp_included_
#define gdal_drivers_mask_hpp_included_

#include <gdal_priv.h>

#include <memory>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "imgproc/rastermask/mappedqtree.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

/**
 * @brief GttDataset
 */

class MaskDataset : public GDALDataset {
public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~MaskDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

private:
    MaskDataset(const fs::path &path, std::ifstream &f);

    class RasterBand;
    friend class RasterBand;
    typedef std::vector<RasterBand> RasterBands;

    typedef imgproc::mappedqtree::RasterMask Mask;
    Mask mask_;

    std::string srs_;
    math::Extents2 extents_;
    math::Size2 tileSize_;

    std::shared_ptr<RasterBands> overviews_;
};

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_MaskDataset(void);
CPL_C_END

#endif // gdal_drivers_mask_hpp_included_
