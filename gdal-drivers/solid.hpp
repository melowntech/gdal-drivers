/**
 * Copyright (c) 2017 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
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
    static ::GDALDataset* Open(GDALOpenInfo *openInfo);

    static ::GDALDataset* CreateCopy(const char *path, ::GDALDataset *src
                                     , int strict, char **options
                                     , GDALProgressFunc progress
                                     , void *progressData);

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
            Band(double value, ::GDALDataType dataType
                 , ::GDALColorInterp colorInterpretation)
                : value(value), dataType(dataType)
                , colorInterpretation(colorInterpretation)
            {}
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
