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

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>
#include <fstream>
#include <iomanip>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"
#include "utility/multivalue.hpp"
#include "utility/streams.hpp"
#include "geo/gdal.hpp"
#include "geo/po.hpp"

#include "blender.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

namespace gdal_drivers {

void writeConfig(const fs::path &file, const BlendingDataset::Config &config)
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(file.string(), std::ios_base::out | std::ios_base::trunc);

    f << "[blender]"
      << "\nsrs = " << config.srs
      << "\nextents = " << config.extents
      << "\noverlap = " << config.overlap
      << "\n\n";

    for (const auto &ds : config.datasets) {
        f << "\n[dataset]"
          << "\npath = " << ds.path
          << "\ninside = " << ds.inside
          << "\n";
    }

    f.close();
}

namespace {

struct ImageReference {
    cv::Rect extents;
    cv::Rect inside;

    ImageReference() = default;
    ImageReference(const cv::Rect &extents, const cv::Rect &inside)
        : extents(extents), inside(inside)
    {}

    typedef std::vector<ImageReference> list;
};

const auto epsilon(1e-6);

bool almostSame(double a, double b)
{
    return (std::abs(a - b) < epsilon);
}

template <typename T>
bool almostSame(const math::Point2_<T> &a, const math::Point2_<T> &b)
{
    return almostSame(a(0), b(0)) && almostSame(a(1), b(1));
}

bool orthogonal(::GDALDataset *ds)
{
    geo::GeoTransform gt;
    ds->GetGeoTransform(gt.data());

    return (std::abs(gt[2]) < epsilon) && (std::abs(gt[4]) < epsilon);
}

math::Point2 getResolution(const geo::GeoTransform &gt)
{
    return { std::hypot(gt[1], gt[4]), std::hypot(gt[2], gt[5]) };
}

math::Point2 getResolution(::GDALDataset *ds)
{
    geo::GeoTransform gt;
    ds->GetGeoTransform(gt.data());
    return getResolution(gt);
}

void checkCompatibility(const fs::path &refPath, ::GDALDataset *ref
                        , const fs::path &dsPath, ::GDALDataset *ds)
{
    // orthogonality
    if (!orthogonal(ds)) {
        LOGTHROW( err2, std::runtime_error )
            << "Non-orthogonal GDAL dataset at " << dsPath
            << " cannot be georeferenced by extents.";
    }

    // resolution
    const auto rRef(getResolution(ref));
    const auto rDs(getResolution(ds));
    if (!almostSame(rRef, rDs)) {
        LOGTHROW( err2, std::runtime_error )
            << "GDAL dataset at " << dsPath << " has different resolution "
            << "(" << rDs
            << ") than reference raster dataset at " << refPath
            << " (" << rRef << ").";
    }

    // band compatibility
    if (ref->GetRasterCount() != ds->GetRasterCount()) {
        LOGTHROW( err2, std::runtime_error )
            << "GDAL dataset at " << dsPath << " has different number "
            << "of raster bands (" << ds->GetRasterCount()
            << ") than reference raster dataset at " << refPath
            << " (" << ref->GetRasterCount() << ").";
    }

    // TODO: check color interpretation
    // TODO: check data type
}

struct Descriptor {
    math::Extents2 extents;
    math::Size2 size;

    Descriptor() = default;
    Descriptor(::GDALDataset *ds);
    typedef std::vector<Descriptor> list;
};


Descriptor::Descriptor(::GDALDataset *ds)
    : size(ds->GetRasterXSize(), ds->GetRasterYSize())
{
    geo::GeoTransform gt;
    ds->GetGeoTransform(gt.data());

    const auto &transform([&](double x, double y)
    {
        const auto &d(gt.data());
        return math::Point2(d[0] + x * d[1] + y * d[2]
                            , d[3] + x * d[4] + y * d[5]);
    });

    const auto ll(transform(0, size.height));
    const auto lr(transform(size.width, size.height));
    const auto ul(transform(0, 0));
    const auto ur(transform(size.width, 0));

    extents.ll(0) = std::min({ll(0), lr(0), ul(0), ur(0)});
    extents.ll(1) = std::min({ll(1), lr(1), ul(1), ur(1)});
    extents.ur(0) = std::max({ll(0), lr(0), ul(0), ur(0)});
    extents.ur(1) = std::max({ll(1), lr(1), ul(1), ur(1)});
}

} // namespace

/**
 * @brief BorderedAreaRasterBand
 */
class BlendingDataset::RasterBand : public ::GDALRasterBand {
public:
    RasterBand(BlendingDataset *dset, int bandIndex
               , const ImageReference::list &references);

    virtual ~RasterBand() {}

    virtual CPLErr IReadBlock(int nBlockXOff, int nBlockYOff, void *image);

    virtual GDALColorInterp GetColorInterpretation() {
        return bands_.front().band->GetColorInterpretation();
    }

    struct Band {
        typedef std::vector<Band> list;

        ::GDALRasterBand *band;
        ImageReference ref;

        Band(::GDALRasterBand *band, const ImageReference &ref)
            : band(band), ref(ref)
        {}
    };

private:
    /** List of source bands.
     */
    Band::list bands_;
};

BlendingDataset::BlendingDataset(const Config &config)
    : config_(config)
    , srs_(config.srs.as(geo::SrsDefinition::Type::wkt).srs)
{
    // open all datasets
    ::GDALDataset *main{};
    Descriptor des;
    Descriptor::list descriptors;

    for (auto &ds : config.datasets) {
        std::unique_ptr< ::GDALDataset> dset
            (static_cast< ::GDALDataset*>
             (::GDALOpen(ds.path.c_str(), GA_ReadOnly)));

        if (!dset) {
            LOGTHROW( err2, std::runtime_error )
                << "Failed to open dataset " << ds.path << ".";
        }

        descriptors.emplace_back(dset.get());

        if (datasets_.empty()) {
            main = dset.get();
            des = descriptors.back();
        } else {
            checkCompatibility(config.datasets.front().path, main
                               , ds.path, dset.get());
        }

        datasets_.push_back(std::move(dset));
    }

    ImageReference::list references;

    math::Point2 resolution;
    math::Point2 origin;

    {
        // align extents with dataset
        geo::GeoTransform gt;
        main->GetGeoTransform(gt.data());

        origin(0) = gt[0];
        origin(1) = gt[3];
        resolution = getResolution(gt);
    }

    const auto &align([&](const math::Extents2 &extents)
    {
        math::Extents2 res(extents.ll - origin, extents.ur - origin);
        for (int i = 0; i < 2; ++i) {
            res.ll(i) = ((std::floor(res.ll(i) / resolution(i))
                          * resolution(i) + origin(i)));
            res.ur(i) = ((std::floor(res.ur(i) / resolution(i))
                          * resolution(i) + origin(i)));
        }
        return res;
    });

    // align extents
    const auto extents(align(config.extents));
    // LOG(info4) << std::fixed << "extents: " << extents;

    // compute raster size and set geo transform
    {
        const auto es(math::size(extents));
        nRasterXSize = es.width / resolution(0);
        nRasterYSize = es.height / resolution(1);

        geoTransform_[0] = extents.ll(0);
        geoTransform_[1] = resolution(0);
        geoTransform_[2] = 0.0;

        geoTransform_[3] = extents.ur(1);
        geoTransform_[4] = 0.0;
        geoTransform_[5] = -resolution(1);
    }

    const auto &point2pixel([&](const math::Point2 &p)
    {
        const math::Point2 shiftd(ul(extents) - p);
        return math::Point2i(int(std::round(shiftd(0) / resolution(0)))
                             , -int(std::round(shiftd(1) / resolution(1))));
    });

    const auto &pixelExtents([&](const math::Extents2 &e
                                 , const math::Size2 &size)
    {
        const auto ll(point2pixel(ul(e)));
        return cv::Rect(ll(0), ll(1), size.width, size.height);
    });

    const auto &pixelInside([&](const math::Extents2 &e)
    {
        const auto ll(point2pixel(ul(e)));
        const auto ur(point2pixel(lr(e)));
        return cv::Rect(ll(0), ll(1), ll(0) - ur(0), ll(1) - ur(1));
    });

    // compute references
    auto idescriptors(descriptors.begin());
    for (const auto &ds : config_.datasets) {
        const auto &des(*idescriptors++);

        // LOG(info4) << std::fixed << "extents: " << des.extents;
        // LOG(info4) << std::fixed << "inside:  " << ds.inside;

        references.emplace_back(pixelExtents(des.extents, des.size)
                                , pixelInside(align(ds.inside)));

        const auto &ref(references.back());
        LOG(info4) << std::fixed << "px extents: " << ref.extents;
        LOG(info4) << std::fixed << "px inside:  " << ref.inside;
    }

    // create bands

    std::size_t bandCount(main->GetRasterCount());
    for (std::size_t band(1); band <= bandCount; ++band) {
        SetBand(band, new RasterBand(this, band, references));
    }
}

CPLErr BlendingDataset::GetGeoTransform(double *padfTransform)
{
    std::copy(geoTransform_.begin(), geoTransform_.end()
              , padfTransform);
    return CE_None;
}

const char* BlendingDataset::GetProjectionRef()
{
    return srs_.c_str();
}

BlendingDataset::RasterBand
::RasterBand(BlendingDataset *dset, int bandIndex
             , const ImageReference::list &references)
{
    bands_.reserve(dset->datasets_.size());
    auto ireferences(references.begin());
    for (const auto &ds : dset->datasets_) {
        bands_.emplace_back(ds->GetRasterBand(bandIndex), *ireferences++);
    }

    nRasterXSize = dset->nRasterXSize;
    nRasterYSize = dset->nRasterXSize;

    nBlockXSize = 256;
    nBlockYSize = 256;
    eDataType = bands_.front().band->GetRasterDataType();
}

namespace {

int gdal2cv(GDALDataType type)
{
    switch (type) {
    case GDT_Byte: return CV_8UC1;
    case GDT_UInt16: return CV_16UC1;
    case GDT_Int16: return CV_16SC1;
    case GDT_UInt32: return CV_32SC1;
    case GDT_Int32: return CV_32SC1;
    case GDT_Float32: return CV_32FC1;
    case GDT_Float64: return CV_64FC1;

    default:
        LOGTHROW(err2, std::logic_error)
            << "Unsupported datatype " << type << " in raster.";
    }
    return 0; // never reached
}

} // namespace

CPLErr BlendingDataset::RasterBand::IReadBlock(int nBlockXOff, int nBlockYOff
                                               , void *rawImage)
{
    cv::Rect block(nBlockXOff * nBlockXSize
                   , nBlockYOff * nBlockYSize
                   , nBlockXSize, nBlockYSize);

    LOG(info4) << "reading block: [" << nBlockXOff << ", " << nBlockYOff
               << "]: " << block;

    // image accumulator
    cv::Mat_<double> acc(nBlockXSize, nBlockYSize, 0.0);

    // for each band
    for (auto &band : bands_) {
        // compute source block
        auto roi(block & band.ref.extents);
        if (!roi.area()) { continue; }

        const auto origin(roi.tl() - block.tl());

        LOG(info4) << "extents: " << band.ref.extents;
        LOG(info4) << "block: " << block;
        LOG(info4) << "origin: " << origin;
        LOG(info4) << "roi: " << roi;
        // localize
        const auto local(roi - band.ref.extents.tl());
        LOG(info4) << "local: " << local;
        cv::Mat_<double> tmp(local.size());

        // read block via generic RasterIO
        const auto err
            (band.band->RasterIO(GF_Read
                                 , local.x, local.y
                                 , local.width, local.height
                                 , tmp.data
                                 , local.width, local.height
                                 , GDT_Float64
                                 , tmp.elemSize()
                                 , local.width * tmp.elemSize()
                                 , nullptr));

        if (err != CE_None) { return err; }

        // TODO: add data to accumulator
    }

    // copy data into output image
    {
        const auto type(gdal2cv(bands_.front().band->GetRasterDataType()));
        cv::Mat out(nBlockYSize, nBlockXSize, type, rawImage);
        acc.convertTo(out, type);
    }
    return CE_None;
}

GDALDataset* BlendingDataset::Open(GDALOpenInfo *openInfo)
{
    ::CPLErrorReset();

    po::options_description config("blending GDAL driver");
    po::variables_map vm;
    Config cfg;

    config.add_options()
        ("blender.srs", po::value(&cfg.srs)->required()
         , "SRS definition. Use [WKT], +proj or EPSG:num.")
        ("blender.extents", po::value(&cfg.extents)->required()
         , "Geo extents of dataset (ulx,uly:urx,ury).")
        ("blender.overlap", po::value(&cfg.overlap)->required()
         , "Blending dataset overlap.")
        ;

    using utility::multi_value;

    config.add_options()
        ("dataset.path", multi_value<decltype(Config::Dataset::path)>()
         , "Value to return.")
        ("dataset.inside", multi_value<decltype(Config::Dataset::inside)>()
         , "Data type.")
        ;

    po::basic_parsed_options<char> parsed(&config);

    // try to parse file -> cannot parse -> probably not a solid file format
    try {
        std::ifstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(openInfo->pszFilename);
        f.exceptions(std::ifstream::badbit);
        parsed = (po::parse_config_file(f, config));
        if (parsed.options.empty()) {
            // structure valid but nothing read -> not a solid file
            return nullptr;
        }
    } catch (...) { return nullptr; }

    // no updates
    if (openInfo->eAccess == GA_Update) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "The Blending driver does not support update "
                 "access to existing datasets.\n");
        return nullptr;
    }

    // initialize dataset
    try {
        po::store(parsed, vm);
        po::notify(vm);

        // process bands
        auto &datasets(cfg.datasets);
        datasets.resize(vm["dataset.path"].as<std::vector<fs::path>>().size());
        using utility::process_multi_value;
        process_multi_value(vm, "dataset.path"
                            , datasets, &Config::Dataset::path);
        process_multi_value(vm, "dataset.inside"
                            , datasets, &Config::Dataset::inside);
        return new BlendingDataset(cfg);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "BlendingDataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

std::unique_ptr<BlendingDataset>
BlendingDataset::create(const fs::path &path, const Config &config)
{
    std::unique_ptr<BlendingDataset> solid(new BlendingDataset(config));
    writeConfig(path, config);
    return solid;
}

} // namespace gdal_drivers

/* GDALRegister_BlendingDataset */

void GDALRegister_BlendingDataset()
{
    if (!GDALGetDriverByName("Blender")) {
        std::unique_ptr<GDALDriver> driver(new GDALDriver());

        driver->SetDescription("Blender");
        driver->SetMetadataItem
            (GDAL_DMD_LONGNAME
             , "Driver that blends multiple datasets into one.");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        driver->pfnOpen = gdal_drivers::BlendingDataset::Open;

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
