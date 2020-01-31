/**
 * Copyright (c) 2019 Melown Technologies SE
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

#ifndef gdal_drivers_blender_hpp_included_
#define gdal_drivers_blender_hpp_included_

#include <gdal_priv.h>

#include <memory>
#include <array>
#include <vector>
#include <iosfwd>

#include <boost/optional.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"
#include "geo/geotransform.hpp"

namespace gdal_drivers {

/**
 * @brief GttDataset
 */

namespace detail {

void closeGdalDataset(::GDALDataset *ds);

} // namespace detail

class BlendingDataset : public ::GDALDataset {
public:
    static ::GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~BlendingDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();
    virtual int CloseDependentDatasets();

    class Config {
    public:
        struct Dataset {
            boost::filesystem::path path;
            math::Extents2 valid;

            typedef std::vector<Dataset> list;

            Dataset() = default;
            Dataset(const boost::filesystem::path &path
                    , const math::Extents2 &valid)
                : path(path), valid(valid)
            {}

            bool operator==(const Dataset &o) const {
                return (path == o.path) && (valid == o.valid);
            }
        };

        boost::optional<geo::SrsDefinition> srs;
        boost::optional< ::GDALDataType> type;
        math::Extents2 extents;
        double overlap = 0;
        Dataset::list datasets;
        boost::optional<math::Size2f> resolution;
        boost::optional<double> nodata;
    };

    /** Creates new blending dataset and returns open interface.
     */
    static std::unique_ptr<BlendingDataset>
    create(const boost::filesystem::path &path, const Config &config);

    /** Creates blending dataset without writing it to disk.
     */
    static std::unique_ptr<BlendingDataset>
    create(const Config &config);

    std::unique_ptr<BlendingDataset>
    create(const std::string &config);

    typedef std::unique_ptr< ::GDALDataset
                             , decltype(&detail::closeGdalDataset)> Dataset;
    typedef std::vector<Dataset> Datasets;

private:
    BlendingDataset(const Config &config);

    class RasterBand;
    friend class RasterBand;
    typedef std::vector<RasterBand> RasterBands;

    Config config_;
    std::string srs_;
    geo::GeoTransform geoTransform_;

    math::Size2f overlap_;

    Datasets datasets_;
};

void writeConfig(std::ostream &os, const BlendingDataset::Config &config);

void writeConfig(const boost::filesystem::path &file
                 , const BlendingDataset::Config &config);

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_BlendingDataset(void);
CPL_C_END

#endif // gdal_drivers_blender_hpp_included_
