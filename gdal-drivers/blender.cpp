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
#include <numeric>
#include <vector>
#include <iterator>
#include <fstream>
#include <iomanip>

#include <boost/algorithm/string/predicate.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/raise.hpp"
#include "utility/multivalue.hpp"
#include "utility/streams.hpp"

#include "geo/gdal.hpp"
#include "geo/cv.hpp"

#include "blender.hpp"

namespace po = boost::program_options;
namespace ba = boost::algorithm;
namespace fs = boost::filesystem;

namespace gdal_drivers {

namespace detail {

void closeGdalDataset(::GDALDataset *ds) { ::GDALClose(ds); }

} // namespace detail

void writeConfig(std::ostream &f, const BlendingDataset::Config &config)
{
    f << "[blender]"
      << "\nextents = " << config.extents
      << "\noverlap = " << config.overlap
        ;

    if (config.srs) {
        f << "\nsrs = " << config.srs.value();
    }

    if (config.type) {
        f << "\ntype = " << config.type.value();
    }

    if (config.resolution) {
        f << "\nresolution = " << config.resolution.value();
    }

    if (config.nodata) {
        f << "\nnodata = " << config.nodata.value();
    }

    f << "\n\n";

    for (const auto &ds : config.datasets) {
        f << "\n[dataset]"
          << "\npath = " << ds.path
          << "\nvalid = " << ds.valid
          << "\n";
    }

}

void writeConfig(const fs::path &file, const BlendingDataset::Config &config)
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(file.string(), std::ios_base::out | std::ios_base::trunc);
    f << std::fixed;
    writeConfig(f, config);
    f.close();
}

namespace {

struct ImageReference {
    fs::path path;
    cv::Rect extents;
    cv::Rect2d valid;

    ImageReference() = default;
    ImageReference(const fs::path &path, const cv::Rect &extents
                   , const cv::Rect2d &valid)
        : path(path), extents(extents), valid(valid)
    {}

    typedef std::vector<ImageReference> list;
};

const auto epsilon(1e-4);

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
        LOGTHROW(err2, std::runtime_error)
            << "Non-orthogonal GDAL dataset at " << dsPath
            << " cannot be georeferenced by extents.";
    }

    // resolution
    const auto rRef(getResolution(ref));
    const auto rDs(getResolution(ds));
    if (!almostSame(rRef, rDs)) {
        LOGTHROW(err2, std::runtime_error)
            << "GDAL dataset at " << dsPath << " has different resolution "
            << "(" << rDs
            << ") than reference raster dataset at " << refPath
            << " (" << rRef << ").";
    }

    // band compatibility
    if (ref->GetRasterCount() != ds->GetRasterCount()) {
        LOGTHROW(err2, std::runtime_error)
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
    math::Point2d resolution;

    Descriptor() = default;
    Descriptor(::GDALDataset *ds);
    typedef std::vector<Descriptor> list;
};


Descriptor::Descriptor(::GDALDataset *ds)
    : size(ds->GetRasterXSize(), ds->GetRasterYSize())
    , resolution(getResolution(ds))
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

using Image = cv::Mat_<double>;
using Mask = cv::Mat_<std::uint8_t>;

struct Locator {
    cv::Rect roi;
    cv::Rect local;
    cv::Rect view;

    Locator(const cv::Rect &block, const cv::Rect &extents)
        : roi(block & extents)
        , local(roi - extents.tl())
        , view(roi.x - block.x, roi.y - block.y, local.width, local.height)
    {}

    operator bool() const { return roi.area(); }
};

template <typename T>
CPLErr loadImage(cv::Mat_<T> &image, const Locator &l, ::GDALRasterBand &band)
{
    image.create(l.local.size());
    return band.RasterIO(GF_Read
                         , l.local.x, l.local.y
                         , l.local.width, l.local.height
                         , image.data
                         , l.local.width, l.local.height
                         , geo::cv2gdal(image.depth())
                         , image.elemSize(), image.step
                         , nullptr);
}

/** No normalization by default
 */
template <typename T> void normalizeMask(cv::Mat_<T>&) {}

inline void normalizeMask(Image &image) {
    for (auto &px : image) { if (px) { px = 1.0; } }
}

template <typename T>
void fullMask(cv::Mat_<T> &mask, const cv::Size &size) {
    mask = cv::Mat_<T>(size, T(255));
}

inline void fullMask(Image &mask, const cv::Size &size) {
    mask = Image(size, Image::value_type(1.0));
}

template <typename T>
CPLErr loadMask(cv::Mat_<T> &mask, const Locator &l, ::GDALRasterBand &band)
{
    if (band.GetMaskFlags() & GMF_ALL_VALID) {
        // all valid
        fullMask(mask, l.local.size());
        return CE_None;
    }

    // not all pixels valid, load mask from mask band
    const auto err = loadImage(mask, l, *band.GetMaskBand());
    if (err != CE_None) { return err; }

    normalizeMask(mask);
    return CE_None;
}

} // namespace

/** BorderedAreaRasterBand
 *
 * TODO: add suppot for per-dataset mask and ditch mask altogether if there is
 * no input mask at all.
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

    virtual double GetNoDataValue(int *pbSuccess) {
        if (pbSuccess) { *pbSuccess = bool(nodata_); }
        return (nodata_ ? nodata_.value() : 0.0);
    }

    virtual GDALColorTable* GetColorTable() {
        return colorTable_.get();
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
    CPLErr maskIReadBlock(int nBlockXOff, int nBlockYOff, void *image);

    class MaskBand : public ::GDALRasterBand {
    public:
        MaskBand(RasterBand *owner);

        virtual CPLErr IReadBlock(int nBlockXOff, int nBlockYOff
                                  , void *image)
        {
            return owner_->maskIReadBlock(nBlockXOff, nBlockYOff, image);
        }

    private:
        RasterBand *owner_;
    };

    /** List of source bands.
     */
    Band::list bands_;
    boost::optional<double> nodata_;
    std::unique_ptr< ::GDALColorTable> colorTable_;
    math::Size2f overlap_;
};

BlendingDataset::BlendingDataset(const Config &config)
    : config_(config)
{
    // open all datasets
    ::GDALDataset *main{};
    Descriptor des;
    Descriptor::list descriptors;

    for (auto &ds : config.datasets) {
        Dataset dset
            (static_cast< ::GDALDataset*>
             (::GDALOpen(ds.path.c_str(), GA_ReadOnly))
             , &detail::closeGdalDataset);

        if (!dset) {
            LOGTHROW(err2, std::runtime_error)
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

    // use provided srs or main dataset one
    if (config.srs) {
        srs_ = config.srs->as(geo::SrsDefinition::Type::wkt).srs;
    } else {
        srs_ = main->GetProjectionRef();
    }

    {
        // align extents with dataset
        geo::GeoTransform gt;
        main->GetGeoTransform(gt.data());

        origin(0) = gt[0];
        origin(1) = gt[3];

        if (config.resolution) {
            // use provided resolution, no check is done
            resolution = { config.resolution->width
                           , config.resolution->height };
        } else {
            // use first dataset resolution
            resolution = getResolution(gt);
        }
    }

    const auto &align([&](const math::Extents2 &extents)
    {
        math::Extents2 res(extents.ll - origin, extents.ur - origin);
        for (int i = 0; i < 2; ++i) {
            res.ll(i) = ((std::floor(res.ll(i) / resolution(i))
                          * resolution(i) + origin(i)));
            res.ur(i) = ((std::ceil(res.ur(i) / resolution(i))
                          * resolution(i) + origin(i)));
        }
        return res;
    });

    // align extents
    const auto extents(align(config.extents));

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

        overlap_.width = std::round(config.overlap / resolution(0));
        overlap_.height = std::round(config.overlap / resolution(1));
    }

    const auto &point2pixel([&](const math::Point2 &p
                                , const math::Point2d &res)
    {
        const math::Point2 shiftd(p - ul(extents));
        return math::Point2i(shiftd(0) / res(0), -shiftd(1) / res(1));
    });

    const auto &pixelExtents([&](const math::Extents2 &e
                                 , const math::Size2 &size
                                 , const math::Point2d &res)
    {
        const auto ll(point2pixel(ul(e), res));
        return cv::Rect(ll(0), ll(1), size.width, size.height);
    });

    const auto &point2pixeld([&](const math::Point2d &p
                                 , const math::Point2d &res)
    {
        const math::Point2d shiftd(p - ul(extents));
        return math::Point2d(shiftd(0) / res(0), -shiftd(1) / res(1));
    });

    const auto &pixelValid([&](const math::Extents2 &e
                               , const math::Point2d &res)
    {
        const auto ll(point2pixeld(ul(e), res));
        const auto ur(point2pixeld(lr(e), res));
        return cv::Rect2d(ll(0), ll(1), ur(0) - ll(0), ur(1) - ll(1));
    });

    // compute references
    auto idescriptors(descriptors.begin());
    for (const auto &ds : config_.datasets) {
        const auto &des(*idescriptors++);

        references.emplace_back(ds.path
                                , pixelExtents(des.extents, des.size
                                               , resolution)
                                , pixelValid(ds.valid, resolution));
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

int BlendingDataset::CloseDependentDatasets()
{
    if (datasets_.empty()) { return FALSE; }
    datasets_.clear();
    return TRUE;
}

BlendingDataset::RasterBand
::RasterBand(BlendingDataset *dset, int bandIndex
             , const ImageReference::list &references)
    : nodata_(dset->config_.nodata)
    , overlap_(dset->overlap_)
{
    bands_.reserve(dset->datasets_.size());
    auto ireferences(references.begin());
    for (const auto &ds : dset->datasets_) {
        bands_.emplace_back(ds->GetRasterBand(bandIndex), *ireferences++);
    }

    // copy color table from first dataset (if any)
    if (auto *colorTable = bands_.front().band->GetColorTable()) {
        colorTable_.reset(colorTable->Clone());
        // discrete data, disable overlap
        overlap_ = {};
    }

    nBand = bandIndex;
    nRasterXSize = dset->nRasterXSize;
    nRasterYSize = dset->nRasterYSize;

    nBlockXSize = 256;
    nBlockYSize = 256;
    eDataType = (dset->config_.type
                 ? *dset->config_.type
                 : bands_.front().band->GetRasterDataType());

    if (!nodata_) {
        poMask = new MaskBand(this);
        bOwnMask = true;
    }
}

BlendingDataset::RasterBand::MaskBand::MaskBand(RasterBand *owner)
    : owner_(owner)
{
    nRasterXSize = owner_->nRasterXSize;
    nRasterYSize = owner_->nRasterYSize;

    nBlockXSize = 256;
    nBlockYSize = 256;
    eDataType = GDT_Byte;
}

CPLErr BlendingDataset::RasterBand
::IReadBlock(int nBlockXOff, int nBlockYOff, void *rawImage)
{
    cv::Rect block(nBlockXOff * nBlockXSize
                   , nBlockYOff * nBlockYSize
                   , nBlockXSize, nBlockYSize);

    Image acc(nBlockYSize, nBlockXSize, 0.0);
    Image wacc(nBlockYSize, nBlockXSize, 0.0);

    // for each band
    for (auto &band : bands_) {
        // compute source block
        Locator l(block, band.ref.extents);
        if (!l) { continue; }

        // read block via generic RasterIO
        Image image;
        {
            const auto err(loadImage(image, l, *band.band));
            if (err != CE_None) { return err; }
        }

        // get weights
        Image weights;
        {
            const auto err(loadMask(weights, l, *band.band));
            if (err != CE_None) { return err; }
        }

        // compute weight for each pixel
        if (math::empty(overlap_)) {
            // no overlap, use only pixels inside the valid image area
            const auto &valid(band.ref.valid);
            cv::Point2f p(l.roi.x + 0.5, l.roi.y + 0.5);
            for (int j = 0; j < weights.rows; ++j, ++p.y) {
                auto pp(p);
                for (int i = 0; i < weights.cols; ++i, ++pp.x) {
                    // invalidate pixel ouside of valid area
                    if (!valid.contains(pp)) { weights(j, i) = 0.0; }
                }
            }
        } else {
            // apply overlap, take into acount pixels ouside the valid image
            // area
            const auto &valid(band.ref.valid);

            // full kernel area
            const auto kernelArea(4 * math::area(overlap_));

            // base kernel at image's upper-left corner
            cv::Rect2d
                k(l.roi.x - overlap_.width + 0.5
                  , l.roi.y - overlap_.height + 0.5
                  , overlap_.width * 2, overlap_.height * 2);

            for (int j = 0; j < weights.rows; ++j, ++k.y) {
                auto kernel(k);
                for (int i = 0; i < weights.cols; ++i, ++kernel.x) {
                    // area of the kernel clipped by the "valid" extents
                    const auto area((valid & kernel).area());
                    // apply weight
                    weights(j, i) *= (area / kernelArea);
                }
            }
        }

        // add weighted data to accumulator
        cv::multiply(image, weights, image);
        Image(acc, l.view) += image;

        // update weight total
        Image(wacc, l.view) += weights;
    }

    // invalid pixel mask (NB: do not use auto, operator returns template
    // expression class)
    cv::Mat invalidMask(wacc == 0.0);

    // set weight for invaid pixels to 1 to not divide by zero
    wacc.setTo(1.0, invalidMask);
    // apply weights total to accumulated image
    acc /= wacc;

    // apply no data if present
    if (nodata_) {
        acc.setTo(nodata_.value(), invalidMask);
    }

    {
        // copy data into the output image
        const auto type(geo::gdal2cv(eDataType));
        cv::Mat out(nBlockYSize, nBlockXSize, type, rawImage);
        acc.convertTo(out, type);
    }
    return CE_None;
}

CPLErr BlendingDataset::RasterBand
::maskIReadBlock(int nBlockXOff, int nBlockYOff, void *rawImage)
{
    cv::Rect block(nBlockXOff * nBlockXSize
                   , nBlockYOff * nBlockYSize
                   , nBlockXSize, nBlockYSize);

    Mask acc(nBlockYSize, nBlockXSize, std::uint8_t(0));

    // for each band
    for (auto &band : bands_) {
        // compute source block
        Locator l(block, band.ref.extents);
        if (!l) { continue; }

        // read block via generic RasterIO
        Image image;
        {
            const auto err(loadImage(image, l, *band.band));
            if (err != CE_None) { return err; }
        }

        // get weights
        Mask mask;
        {
            const auto err(loadMask(mask, l, *band.band));
            if (err != CE_None) { return err; }
        }

        // determine validity status of every pixel

        if (math::empty(overlap_)) {
            // no overlap, use only pixels inside the valid image area
            const auto &valid(band.ref.valid);
            cv::Point2f p(l.roi.x + 0.5, l.roi.y + 0.5);
            for (int j = 0; j < mask.rows; ++j, ++p.y) {
                auto pp(p);
                for (int i = 0; i < mask.cols; ++i, ++pp.x) {
                    // invalidate pixel outside of the valid area
                    if (!valid.contains(pp)) { mask(j, i) = 0; }
                }
            }
        } else {
            // apply overlap, take into acount pixels outside of the valid image
            // area
            const auto &valid(band.ref.valid);

            // base kernel at image's upper-left corner
            cv::Rect2d
                k(l.roi.x - overlap_.width + 0.5
                  , l.roi.y - overlap_.height + 0.5
                  , overlap_.width * 2, overlap_.height * 2);

            for (int j = 0; j < mask.rows; ++j, ++k.y) {
                auto kernel(k);
                for (int i = 0; i < mask.cols; ++i, ++kernel.x) {
                    // area of the kernel clipped by the "valid" extents
                    const auto area((valid & kernel).area());
                    // mask pixel if outside
                    if (!area) { mask(j, i) = 0; }
                }
            }
        }

        // set valid pixels from mask
        Mask(acc, l.view).setTo(255, mask);
    }

    {
        // copy mask into the output image
        cv::Mat out(nBlockYSize, nBlockXSize, CV_8U, rawImage);
        acc.copyTo(out);
    }
    return CE_None;
}

bool loadConfig(BlendingDataset::Config &cfg, std::istream &is
                , bool probe = false)
{
    using Config = BlendingDataset::Config;

    po::options_description config("blending GDAL driver");
    po::variables_map vm;
    config.add_options()
        ("blender.extents", po::value(&cfg.extents)->required()
         , "Geo extents of dataset (ulx,uly:urx,ury).")
        ("blender.overlap", po::value(&cfg.overlap)->required()
         , "Blending dataset overlap.")
        ("blender.srs", po::value<geo::SrsDefinition>()
         , "SRS definition. Use [WKT], +proj or EPSG:num.")
        ("blender.type", po::value< ::GDALDataType>()
         , "Data type (Byte, Int, UInt, etc. Defaultsto first dataset "
         "data type.")
        ("blender.resolution", po::value<math::Size2f>()
         , "Resolution of dataset. Defaults to first dataset resolution.")
        ("blender.nodata", po::value<double>()
         , "Reported nodata value. If not set a per-dataset mask layer "
         "is provided")
        ;

    using utility::multi_value;

    config.add_options()
        ("dataset.path", multi_value<decltype(Config::Dataset::path)>()
         , "Value to return.")
        ("dataset.valid", multi_value<decltype(Config::Dataset::valid)>()
         , "Data type.")
        ;

    po::basic_parsed_options<char> parsed(&config);

    // try to parse file -> cannot parse -> probably not a blending dataset file
    // format
    try {
        parsed = (po::parse_config_file(is, config));
        if (parsed.options.empty()) {
            // structure valid but nothing read -> not a blend file
            if (!probe) {
                CPLError(CE_Failure, CPLE_IllegalArg
                         , "Not a blending dataset.\n");
            }
            return false;
        }
    } catch (...) {
        if (!probe) {
            CPLError(CE_Failure, CPLE_IllegalArg
                     , "Not a blending dataset.\n");
        }
        return false;
    }

    // initialize dataset
    try {
        po::store(parsed, vm);
        po::notify(vm);

        if (vm.count("blender.srs")) {
            cfg.srs = vm["blender.srs"].as<geo::SrsDefinition>();
        }

        if (vm.count("blender.type")) {
            cfg.type = vm["blender.type"].as< ::GDALDataType>();
        }

        if (vm.count("blender.resolution")) {
            cfg.resolution = vm["blender.resolution"].as<math::Size2f>();
        }

        if (vm.count("blender.nodata")) {
            cfg.nodata = vm["blender.nodata"].as<double>();
        }

        // process bands
        auto &datasets(cfg.datasets);
        datasets.resize(vm["dataset.path"].as<std::vector<fs::path>>().size());
        using utility::process_multi_value;
        process_multi_value(vm, "dataset.path"
                            , datasets, &Config::Dataset::path);
        process_multi_value(vm, "dataset.valid"
                            , datasets, &Config::Dataset::valid);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "BlendingDataset initialization failure (%s).\n", e.what());
        return false;
    }

    return true;
}

/** Loads config from file. Used path only if non-null. If path is non-null then
 *  it is a hard error to fail to load from it.
 */
bool configFromFile(BlendingDataset::Config &cfg, GDALOpenInfo *openInfo
                    , const char *path = nullptr)
{
    try {
        std::ifstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(path ? path : openInfo->pszFilename);
        f.exceptions(std::ifstream::badbit);

        // if path is not give -> just probe
        if (!loadConfig(cfg, f, !path)) { return false; }
    } catch (...) {
        if (path) {
            CPLError(CE_Failure, CPLE_IllegalArg
                     , "Not a blending dataset.\n");
        }
        return false;
    }

    // no updates
    if (openInfo->eAccess == GA_Update) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "The Blending driver does not support update "
                 "access to existing datasets.\n");
        return false;
    }

    return true;
}

bool loadFromPointer(BlendingDataset::Config &cfg, GDALOpenInfo*
                     , const char *spec)
{
    // load raw pointer as an hex integer
    std::istringstream is(spec);
    std::uintptr_t raw;
    if (!(is >> std::hex >> raw)) {
        CPLError(CE_Failure, CPLE_IllegalArg,
                 "Bleding driver: Missing config pointer value.\n");
        return false;
    }

    if (!raw) {
        CPLError(CE_Failure, CPLE_IllegalArg,
                 "Bleding driver: Invalid config pointer value.\n");
        return false;
    }

    // copy from parameter
    cfg = *reinterpret_cast<const BlendingDataset::Config*>(raw);
    return true;
}

bool loadConfig(BlendingDataset::Config &cfg, GDALOpenInfo *openInfo)
{
    const std::string blenderPrefix("blender:");
    const std::string ptrPrefix("blender:ptr=");
    if (ba::istarts_with(openInfo->pszFilename, ptrPrefix)) {
        return loadFromPointer(cfg, openInfo
                               , openInfo->pszFilename + ptrPrefix.size());
    } else if (ba::istarts_with(openInfo->pszFilename, blenderPrefix)) {
        return configFromFile
            (cfg, openInfo, openInfo->pszFilename + blenderPrefix.size());
    }
    return configFromFile(cfg, openInfo);
}

GDALDataset* BlendingDataset::Open(GDALOpenInfo *openInfo)
{
    ::CPLErrorReset();

    Config cfg;
    if (!loadConfig(cfg, openInfo)) { return nullptr; }

    // initialize dataset
    try {
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
    std::unique_ptr<BlendingDataset> ds(new BlendingDataset(config));
    writeConfig(path, config);
    return ds;
}

std::unique_ptr<BlendingDataset>
BlendingDataset::create(const Config &config)
{
    return std::unique_ptr<BlendingDataset>(new BlendingDataset(config));
}

std::unique_ptr<BlendingDataset>
BlendingDataset::create(const std::string &config)
{
    Config cfg;
    std::istringstream is(config);
    if (!loadConfig(cfg, is)) {
        LOGTHROW(err2, std::runtime_error)
            << "Unable to load BlendingDataset config from string.";
    }

    return std::unique_ptr<BlendingDataset> (new BlendingDataset(cfg));
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
