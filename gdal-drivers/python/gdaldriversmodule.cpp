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
    {
        // make sure we have numpy
        imgproc::py::importNumpy();
    }

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

bp::object readDataset(const BlendingDataset &ds)
{
    /** Load all data from underlying dataset as-is and wrap in ND array
     *  Do not reorder channels.
     */
    geo::GeoDataset::ReadOptions options;
    options.channelsAsIs = true;
    return imgproc::py::asNumpyArray(ds.dset.readData(-1, 0, options));
}

} } // namespace gdal_drivers::py

BOOST_PYTHON_MODULE(melown_gdaldrivers)
{
    using namespace bp;
    namespace py = gdal_drivers::py;

    // blender
    auto BlendingDataset = class_<py::BlendingDataset, boost::noncopyable>
        ("BlendingDataset", init<const py::BlendingDataset::Config&>())

        .def("read", &py::readDataset)

        .def("gdal", &py::openGdal)
        .staticmethod("gdal")
        .def("rasterio", &py::openRasterio)
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
