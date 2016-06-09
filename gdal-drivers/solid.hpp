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
#include <array>
#include <vector>

#include <boost/variant.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"
#include "geo/geotransform.hpp"

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

    class Config {
    public:
        geo::SrsDefinition srs;
        math::Size2 size;
        math::Size2 tileSize;

        struct Band {
            double value;
            ::GDALDataType dataType;
            ::GDALColorInterp colorInterpretation;

            typedef std::vector<Band> list;

            Band() : value() {}
        };
        Band::list bands;

        Config() : tileSize(256, 256) {}

        void extents(const math::Extents2 &e) {
            extentsOrGeoTransform = e;
        }
        const math::Extents2* extents() const;

        void geoTransform(const geo::GeoTransform &g) {
            extentsOrGeoTransform = g;
        }
        const geo::GeoTransform* geoTransform() const;

    private:
        boost::variant<math::Extents2, geo::GeoTransform>
        extentsOrGeoTransform;
    };

    /** Creates new solid dataset and return pointer to it.
     */
    static std::unique_ptr<SolidDataset>
    create(const boost::filesystem::path &path, const Config &config);

private:
    SolidDataset(const Config &config);

    class RasterBand;
    friend class RasterBand;
    typedef std::vector<RasterBand> RasterBands;

    Config config_;
    std::string srs_;
    geo::GeoTransform geoTransform_;
};

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_SolidDataset(void);
CPL_C_END

#endif // gdal_drivers_solid_hpp_included_
