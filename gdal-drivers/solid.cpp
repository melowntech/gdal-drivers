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
/*
 * @file solid.cpp
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

#include "detail/geotransform.hpp"

#include "solid.hpp"

namespace po = boost::program_options;

namespace gdal_drivers {

void writeConfig(const boost::filesystem::path &file
                 , const SolidDataset::Config &config)
{
    std::ofstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);
    f.open(file.string(), std::ios_base::out | std::ios_base::trunc);

    f << std::scientific << std::setprecision(16);

    f << "[solid]"
      << "\nsrs = " << config.srs
      << "\nsize = " << config.size
      << "\ntileSize = " << config.tileSize
        ;

    if (const auto *extents = config.extents()) {
        f << "\nextents = " << *extents;
    } if (const auto *geoTransform = config.geoTransform()) {
        f << "\ngeoTransform = " << detail::GeoTransformWrapper{*geoTransform};
    } else {
        LOGTHROW(err1, std::runtime_error)
            << "Neither extents nor geoTransform are set.";
    }
    f << "\n\n";

    f.unsetf(std::ios_base::floatfield);
    for (const auto &band : config.bands) {
        f << "\n[band]"
          << "\nvalue = " << band.value
          << "\ndataType = " << band.dataType
          << "\ncolorInterpretation = " << band.colorInterpretation
          << "\n";
    }

    f.close();
}

/**
 * @brief BorderedAreaRasterBand
 */
class SolidDataset::RasterBand : public ::GDALRasterBand {
public:
    RasterBand(SolidDataset *dset, const Config::Band &band
               , const std::vector<math::Size2> &overviews);

    virtual ~RasterBand() {
        for (auto &ovr : ovrBands_) { delete ovr; }
    }

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual GDALColorInterp GetColorInterpretation() {
        return colorInterpretation_;
    }

    virtual int GetOverviewCount() { return overviews_.size(); }

    virtual GDALRasterBand* GetOverview(int index) {
        if (index >= int(ovrBands_.size())) { return nullptr; }
        auto &ovr(ovrBands_[index]);
        if (!ovr) {
            ovr = new OvrBand(this, overviews_[index]);
        }
        return ovr;
    }

private:
    class OvrBand : public ::GDALRasterBand {
    public:
        OvrBand(RasterBand *owner, const math::Size2 &size)
            : owner_(owner)
        {
            poDS = owner->poDS;
            nBand = owner->nBand;
            nBlockXSize = owner->nBlockXSize;
            nBlockYSize = owner->nBlockYSize;
            eDataType = owner->eDataType;

            nRasterXSize = size.width;
            nRasterYSize = size.height;
        }

        virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image) {
            return owner_->IReadBlock(blockCol, blockRow, image);
        }

        virtual GDALColorInterp GetColorInterpretation() {
            return owner_->GetColorInterpretation();
        }

    private:
        RasterBand *owner_;
    };

    friend class OvrBand;

    template <typename T>
    void createBlock(const T &value, std::size_t count)
    {
        auto *block(new T[count]);
        std::fill_n(block, count, value);
        block_ = std::shared_ptr<T>(block, [](T *block) { delete [] block; });
        blockSize_ = count * sizeof(T);
    }

    /** Block of pregenerated data.
     */
    std::shared_ptr<void> block_;

    /** Size of block of pregenerated data in bytes.
     */
    std::size_t blockSize_;

    ::GDALColorInterp colorInterpretation_;

    std::vector<math::Size2> overviews_;
    std::vector< ::GDALRasterBand*> ovrBands_;
};

GDALDataset* SolidDataset::Open(GDALOpenInfo *openInfo)
{
    ::CPLErrorReset();

    po::options_description config("solid color GDAL driver");
    po::variables_map vm;
    Config cfg;

    config.add_options()
        ("solid.srs", po::value(&cfg.srs)->required()
         , "SRS definition. Use [WKT], +proj or EPSG:num.")
        ("solid.size", po::value(&cfg.size)->required()
         , "Size of dataset (WxH).")
        ("solid.extents", po::value<math::Extents2>()
         , "Geo extents of dataset (ulx,uly:urx,ury).")
        ("solid.geoTransform", po::value<detail::GeoTransformWrapper>()
         , "Geo transform matrix (m00, m01, m02, m10, m11, m12).")
        ("solid.tileSize", po::value(&cfg.tileSize)
         ->default_value(math::Size2(256, 256))->required()
         , "Tile size.")
        ;

    using utility::multi_value;

    config.add_options()
        ("band.value", multi_value<decltype(Config::Band::value)>()
         , "Value to return.")
        ("band.dataType", multi_value<decltype(Config::Band::dataType)>()
         , "Data type.")
        ("band.colorInterpretation"
         , multi_value<decltype(Config::Band::colorInterpretation)>()
         , "Color interpretation.")
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
                 "The Quadtree Solid driver does not support update "
                 "access to existing datasets.\n");
        return nullptr;
    }

    // initialize dataset
    try {
        po::store(parsed, vm);
        po::notify(vm);

        bool hasExtents(vm.count("solid.extents"));
        bool hasGeoTransform(vm.count("solid.geoTransform"));

        if (hasExtents && hasGeoTransform) {
            CPLError(CE_Failure, CPLE_IllegalArg
                     , "SolidDataset initialization failure:"
                     " both extents and geoTransform are set.\n");
            return nullptr;
        }
        if (!hasExtents && !hasGeoTransform) {
            CPLError(CE_Failure, CPLE_IllegalArg
                     , "SolidDataset initialization failure:"
                     " both extents and geoTransform are unset.\n");
            return nullptr;
        }

        if (hasExtents) {
            cfg.extents(vm["solid.extents"].as<math::Extents2>());
        } else {
            cfg.geoTransform
                (vm["solid.geoTransform"].as<detail::GeoTransformWrapper>()
                 .value);
        }

        // process bands
        auto &bands(cfg.bands);
        bands.resize(vm["band.value"].as<std::vector<double> >().size());
        using utility::process_multi_value;
        process_multi_value(vm, "band.value"
                            , bands, &Config::Band::value);
        process_multi_value(vm, "band.dataType"
                            , bands, &Config::Band::dataType);
        process_multi_value(vm, "band.colorInterpretation"
                            , bands, &Config::Band::colorInterpretation);
        return new SolidDataset(cfg);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "SolidDataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

GDALDataset* SolidDataset::CreateCopy(const char *path
                                      , GDALDataset *src
                                      , int strict
                                      , char **options
                                      , GDALProgressFunc, void *)
{
    auto bands(src->GetRasterCount());

    std::vector<double> colors(bands, 0);
    {
        auto **rawColors(CSLFetchNameValueMultiple(options, "COLOR"));
        for (auto &color : colors) {
            if (!rawColors) { break; }

            try {
                color = boost::lexical_cast<double>(*rawColors);
            } catch (...) {
                CPLError(CE_Failure, CPLE_IllegalArg
                         , "SolidDataset create failure:"
                         " invalid color value %s.\n", *rawColors);
                return nullptr;
            }

            ++rawColors;
        }
    }

    Config config;
    // copy SRS
    config.srs = geo::SrsDefinition
        (src->GetProjectionRef(), geo::SrsDefinition::Type::wkt);

    // copy raster size
    config.size.width = src->GetRasterXSize();
    config.size.height = src->GetRasterYSize();

    // copy geo transformation
    geo::GeoTransform gt;
    src->GetGeoTransform(gt.data());
    config.geoTransform(gt);

    // copy bands
    for (int i(1); i <= bands; ++i) {
        auto *b(src->GetRasterBand(i));
        config.bands.emplace_back(colors[i - 1], b->GetRasterDataType()
                                  , b->GetColorInterpretation());
    }

    writeConfig(path, config);
    return new SolidDataset(config);

    (void) strict;
}

SolidDataset::SolidDataset(const Config &config)
    : SrsHoldingDataset(config.srs)
    , config_(config)
{
    if (const auto *extents = config_.extents()) {
        const auto &e(*extents);
        auto es(math::size(e));

        geoTransform_[0] = e.ll(0);
        geoTransform_[1] = es.width / nRasterXSize;
        geoTransform_[2] = 0.0;

        geoTransform_[3] = e.ur(1);
        geoTransform_[4] = 0.0;
        geoTransform_[5] = -es.height / nRasterYSize;;
    } else if (const auto *geoTransform = config_.geoTransform()) {
        geoTransform_ = *geoTransform;
    }

    nRasterXSize = config_.size.width;
    nRasterYSize = config_.size.height;

    // prepare overviews
    std::vector<math::Size2> overviews;
    {
        auto size(config_.size);
        auto halve([&]()
        {
            size.width = int(std::round(size.width / 2.0));
            size.height = int(std::round(size.height / 2.0));
        });

        halve();
        while ((size.width >= config_.tileSize.width)
               || (size.height >= config_.tileSize.height))
        {
            overviews.push_back(size);
            halve();
        }
    }

    // NB: bands are 1-based, start with zero, pre-increment before setting band
    int i(0);
    for (const auto &band : config_.bands) {
        SetBand(++i, new RasterBand(this, band, overviews));
    }

}

CPLErr SolidDataset::GetGeoTransform(double *padfTransform)
{
    std::copy(geoTransform_.begin(), geoTransform_.end()
              , padfTransform);
    return CE_None;

    return CE_None;
}

SolidDataset::RasterBand
::RasterBand(SolidDataset *dset , const Config::Band &band
             , const std::vector<math::Size2> &overviews)
    : block_(), blockSize_()
    , colorInterpretation_(band.colorInterpretation)
    , overviews_(overviews)
    , ovrBands_(overviews.size(), nullptr)
{
    const auto &cfg(dset->config_);
    poDS = dset;
    nBand = 1;
    nBlockXSize = cfg.tileSize.width;
    nBlockYSize = cfg.tileSize.height;
    eDataType = band.dataType;

    nRasterXSize = cfg.size.width;
    nRasterYSize = cfg.size.height;

    auto count(math::area(cfg.tileSize));

    switch (eDataType) {
    case ::GDT_Byte:
        createBlock<std::uint8_t>(band.value, count);
        break;

    case ::GDT_UInt16:
        createBlock<std::uint16_t>(band.value, count);
        break;

    case ::GDT_Int16:
        createBlock<std::int16_t>(band.value, count);
        break;

    case ::GDT_UInt32:
        createBlock<std::uint32_t>(band.value, count);
        break;

    case ::GDT_Int32:
        createBlock<std::int32_t>(band.value, count);
        break;

    case ::GDT_Float32:
        createBlock<float>(band.value, count);
        break;

    case ::GDT_Float64:
        createBlock<double>(band.value, count);
        break;

    default:
        utility::raise<std::runtime_error>
            ("Unsupported data type <%s>.", eDataType);
    };
}

CPLErr SolidDataset::RasterBand::IReadBlock(int, int, void *rawImage)
{
    // copy pregenerated data into output image
    std::memcpy(rawImage, block_.get(), blockSize_);
    return CE_None;
}

const math::Extents2* SolidDataset::Config::extents() const
{
    return detail::extents(extentsOrGeoTransform_);
}

const geo::GeoTransform* SolidDataset::Config::geoTransform() const
{
    return detail::geoTransform(extentsOrGeoTransform_);
}

std::unique_ptr<SolidDataset>
SolidDataset::create(const boost::filesystem::path &path, const Config &config)
{
    std::unique_ptr<SolidDataset> solid(new SolidDataset(config));
    writeConfig(path, config);
    return solid;
}

} // namespace gdal_drivers

/* GDALRegister_SolidDataset */

void GDALRegister_SolidDataset()
{
    if (!GDALGetDriverByName("Solid")) {
        std::unique_ptr<GDALDriver> driver(new GDALDriver());

        driver->SetDescription("Solid");
        driver->SetMetadataItem
            (GDAL_DMD_LONGNAME
             , "Driver that returns solid valid in all pixels.");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        driver->pfnOpen = gdal_drivers::SolidDataset::Open;
        driver->pfnCreateCopy = gdal_drivers::SolidDataset::CreateCopy;

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
