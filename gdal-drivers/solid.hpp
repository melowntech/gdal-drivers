/**
 * @file solid.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Virtual GDAL driver that returns solid valid in whole extents.
 */

#ifndef gdal_drivers_solid_hpp_included_
#define gdal_drivers_solid_hpp_included_

#include <gdal_priv.h>

#include <memory>
#include <vector>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

namespace gdal_drivers {

/**
 * @brief GttDataset
 */

class SolidDataset : public GDALDataset {
public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~SolidDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

private:
    struct Config {
        geo::SrsDefinition srs;
        math::Size2 size;
        math::Extents2 extents;
        double value;
        ::GDALDataType dataType;
    };

    SolidDataset(const Config &config);

    class RasterBand;
    friend class RasterBand;
    typedef std::vector<RasterBand> RasterBands;

    Config config_;
    std::string srs_;
    math::Size2 tileSize_;
    std::shared_ptr<RasterBands> overviews_;
};

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_SolidDataset(void);
CPL_C_END

#endif // gdal_drivers_solid_hpp_included_
