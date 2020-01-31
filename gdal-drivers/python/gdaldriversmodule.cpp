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

#include <ogr_core.h>

#include <sstream>
#include <string>
#include <vector>
#include <type_traits>
#include <cstdint>

#include <boost/python.hpp>
#include <boost/python/scope.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/handle.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "dbglog/dbglog.hpp"

#include "geo/geodataset.hpp"
#include "geo/gdal.hpp"
#include "geo/cv.hpp"

#include "imgproc/python/numpy.hpp"

#include "pysupport/package.hpp"
#include "pysupport/class.hpp"
#include "pysupport/enum.hpp"
#include "pysupport/converters.hpp"

#include "../register.hpp"
#include "../blender.hpp"

#include "gdaldriversmodule.hpp"

namespace bp = boost::python;
namespace bpc = boost::python::converter;
namespace fs = boost::filesystem;

namespace gdal_drivers { namespace py {

struct BlendingDataset {
    using Config = gdal_drivers::BlendingDataset::Config;

    BlendingDataset(const Config &config)
        : dset(geo::GeoDataset::use
               (gdal_drivers::BlendingDataset::create(config)))
    {}

    geo::GeoDataset dset;
};

template <typename Enum>
typename std::underlying_type<Enum>::type rawEnumValue(Enum value)
{
    return static_cast<typename std::underlying_type<Enum>::type>(value);
}

bp::object openGdal(const BlendingDataset::Config &config)
{
    auto gdal(bp::import("osgeo.gdal"));

    const auto version
        (bp::extract<std::string>(gdal.attr("__version__"))());

    if (version != GDAL_RELEASE_NAME) {
        ::PyErr_SetString
            (::PyExc_RuntimeError
             , "osgeo.gdal links against different gdal library version");
        bp::throw_error_already_set();
    }

    std::ostringstream os;
    os << "blender:ptr=" << static_cast<const void*>(&config);
    return gdal.attr("OpenEx")
        (os.str().c_str(), rawEnumValue(GA_ReadOnly)
         , std::vector<std::string>{"Blender"});
}

bp::object openRasterio(const BlendingDataset::Config &config)
{
    auto rasterio(bp::import("rasterio"));

    const auto version
        (bp::extract<std::string>(rasterio.attr("gdal_version")())());
    if (version != GDAL_RELEASE_NAME) {
        ::PyErr_SetString
            (::PyExc_RuntimeError
             , "rasterio links against different gdal library version");
        bp::throw_error_already_set();
    }

    std::ostringstream os;
    os << "blender:ptr=" << static_cast<const void*>(&config);
    return rasterio.attr("open")(os.str().c_str(), "r", "Blender");
}

bp::object fetch(const geo::GeoDataset &dset
                 , const boost::optional< ::GDALDataType> &type
                 , bool withMask = false)
{
    /** Load all data from underlying dataset as-is and wrap in ND array
     *  Do not reorder channels.
     */
    geo::GeoDataset::ReadOptions options;
    options.channelsAsIs = true;

    /** Use native dataset's data type or requested one.
     */
    const int depth(type ? geo::gdal2cv(*type) : -1);

    auto data(imgproc::py::asNumpyArray(dset.readData(depth, 0, options)));
    if (!withMask) { return data; }

    auto mask(imgproc::py::asNumpyArray(dset.fetchMask()));
    return bp::make_tuple(data, mask);
}

inline geo::OptionalNodataValue asOptNodata(const geo::NodataValue &nodata)
{
    if (nodata) { return nodata; }
    return boost::none;
}

bp::object readDataset(const BlendingDataset &ds
                       , const boost::optional< ::GDALDataType> &type
                       , bool withMask)
{
    return fetch(ds.dset, type, withMask);
}

const char *readDoc(R"R(Returns whole content of dataset as a single ndarray.

Dimensions are in C-style row-major order:
    * multi-channel datasets: (row, columns, channel)
    * single-channel datasets: (row, column)

Channel order is kept as in the original dataset. If data type is not specified
the the native datatype is used.

If withMask keyword argument is False (default) only content array is
returned. If withMask keyword argument is set to True the tuple of (content,
mask) is retuned instead where mask is Byte ndarray containing validity
information.

Arguments:
    * geo.GDALDataType type: output data type (defaults to source data type
    * withMask: validity mask is read as well (defaults to False)

Returns:
    ndarray or (ndarray, ndarray)
)R");

bp::object warpDataset(const BlendingDataset &ds
                       , const math::Extents2 &extents
                       , const boost::optional<geo::SrsDefinition> &srs
                       , const boost::optional<math::Size2i> &size
                       , const boost::optional< ::GDALDataType> &warpType
                       , const boost::optional< ::GDALDataType> &type
                       , const boost::optional
                       <geo::GeoDataset::Resampling> &resampling
                       , const boost::optional<double> &nodata
                       , bool withMask)
{
    // warp and output types
    const auto wt(warpType ? warpType : type);
    boost::optional< ::GDALDataType> ot;
    if (warpType != type) { ot = type; }

    auto warped(geo::GeoDataset::deriveInMemory
                (ds.dset, srs ? srs.value() : ds.dset.srs()
                 , size, extents, wt, asOptNodata(nodata)));

    ds.dset.warpInto(warped, resampling);
    return fetch(warped, ot, withMask);
}

const char *warpDoc(R"R(Returns whole content of warped dataset

See read() method documentation for output properties.

Data type of the warp result is either warpType or type or dataset's type. Type
of the read result is the same as warp result unless type and warp types are
different.

Arguments:
    * math.Extents2 extents: extents of warped dataset
    * geo.SrsDefinition srs: SRS of warped dataset (defaults to source SRS)
    * math.Size2i size: pixel size of warped dataset (computed if not set)
    * geo.GDALDataType warpType: warp data type (if different from output)
    * geo.GDALDataType type: output data type (defaults to source data type
    * geo.GeoDataset.Resampling resampling: resampling method
                                            (defaults to nerest neighbor)
    * double nodata: override nodata value in output (defaults to
                     source nodata value)
    * withMask: see read() method documentation (default to False)

Returns:
    ndarray or (ndarray, ndarray)
)R");

template <typename T> boost::optional<T> opt() { return boost::optional<T>(); }

std::string Config_repr(const gdal_drivers::BlendingDataset::Config &config)
{
    std::ostringstream os;
    gdal_drivers::writeConfig(os, config);
    return os.str();
}

} } // namespace gdal_drivers::py

BOOST_PYTHON_MODULE(melown_gdaldrivers)
{
    using namespace bp;
    namespace py = gdal_drivers::py;
    using py::opt;

    // blender
    auto BlendingDataset = class_<py::BlendingDataset, boost::noncopyable>
        ("BlendingDataset", init<const py::BlendingDataset::Config&>())

        .def("read", &py::readDataset
             , (bp::arg("type") = opt< ::GDALDataType>()
                , bp::arg("withMask") = false
             )
             , py::readDoc)
        .def("warp", &py::warpDataset
             , (bp::arg("extents")
                , bp::arg("srs") = opt<geo::SrsDefinition>()
                , bp::arg("size") = opt<math::Size2i>()
                , bp::arg("warpType") = opt< ::GDALDataType>()
                , bp::arg("type") = opt< ::GDALDataType>()
                , bp::arg("resampling") = opt<geo::GeoDataset::Resampling>()
                , bp::arg("nodata") = opt<double>()
                , bp::arg("withMask") = false
                )
             , py::warpDoc)

        .def("gdal", &py::openGdal
             , "Opens blending dataset as an osgeo.gdal.Dataset.")
        .staticmethod("gdal")

        .def("rasterio", &py::openRasterio
             , "Opens blending dataset as a rasterio.DatasetReader.")
        .staticmethod("rasterio")
        ;

    {
        bp::scope scope(BlendingDataset);

        auto Config = class_<py::BlendingDataset::Config>
            ("Config", init<const py::BlendingDataset::Config&>())
            .def(init<>())
            .def_readwrite("extents", &py::BlendingDataset::Config::extents)
            .def_readwrite("overlap", &py::BlendingDataset::Config::overlap)
            .add_property("srs"
                          , bp::make_getter
                          (&py::BlendingDataset::Config::srs
                           , bp::return_value_policy<bp::return_by_value>())
                          , bp::make_setter
                          (&py::BlendingDataset::Config::srs
                           , bp::return_value_policy<bp::return_by_value>()))
            .add_property("resolution"
                          , bp::make_getter
                          (&py::BlendingDataset::Config::resolution
                           , bp::return_value_policy<bp::return_by_value>())
                          , bp::make_setter
                          (&py::BlendingDataset::Config::resolution
                           , bp::return_value_policy<bp::return_by_value>()))
            .def_readwrite("datasets", &py::BlendingDataset::Config::datasets)

            .def("__repr__", &py::Config_repr)
            ;

        {
            bp::scope scope(Config);

            auto Dataset = class_<py::BlendingDataset::Config::Dataset>
                ("Dataset"
                 , init<const py::BlendingDataset::Config::Dataset&>())
                .def(init<>())
                .def(init<const fs::path&, const math::Extents2&>())
                .add_property
                ("path"
                 , bp::make_getter
                 (&py::BlendingDataset::Config::Dataset::path
                  , bp::return_value_policy<bp::return_by_value>())
                 , bp::make_setter
                 (&py::BlendingDataset::Config::Dataset::path
                  , bp::return_value_policy<bp::return_by_value>()))
                .def_readwrite("valid"
                               , &py::BlendingDataset::Config::Dataset::valid)
                ;

            {
                bp::scope scope(Dataset);
                class_<py::BlendingDataset::Config::Dataset::list>("list")
                    .def(bp::vector_indexing_suite
                         <py::BlendingDataset::Config::Dataset::list>())
                ;
            }
        }
    }

    // register all custom gdal drivers
    gdal_drivers::registerAll();
}

namespace gdal_drivers { namespace py {
PYSUPPORT_MODULE_IMPORT(gdaldrivers)
} } // namespace gdal_drivers::py
