/*
 * @file solid.cpp
 */

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>
#include <fstream>

#include <boost/filesystem/path.hpp>
#include <boost/logic/tribool_io.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/binaryio.hpp"
#include "geo/gdal.hpp"
#include "geo/po.hpp"

#include "./solid.hpp"

namespace po = boost::program_options;

namespace gdal_drivers {

/**
 * @brief BorderedAreaRasterBand
 */
class SolidDataset::RasterBand : public GDALRasterBand {
public:
    RasterBand(SolidDataset *dset, const math::Size2 &size);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~RasterBand() {};

    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 0; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return GCI_GrayIndex; }

    virtual int GetOverviewCount() {
        const auto &overviews(static_cast<SolidDataset*>(poDS)->overviews_);
        return overviews->size();
    }

    virtual GDALRasterBand* GetOverview(int index) {
        const auto &overviews(static_cast<SolidDataset*>(poDS)->overviews_);
        if (index >= int(overviews->size())) { return nullptr; }
        return &(*overviews)[index];
    }

private:
    template <typename T>
    void fill(T *array)
    {
        auto &dset(*static_cast<SolidDataset*>(poDS));
        const T value(dset.config_.value);
        auto count(math::area(dset.tileSize_));
        std::fill(array, array + count, value);
    }

    math::Size2 size_;
};

GDALDataset* SolidDataset::Open(GDALOpenInfo *openInfo)
{
    ::CPLErrorReset();

    po::options_description config("solid color GDAL driver");
    po::variables_map vm;
    Config cfg;

    config.add_options()
        ("solid.value", po::value(&cfg.value)->required()
         , "Value to return.")
        ("solid.datatype", po::value(&cfg.dataType)->required()
         , "Data type.")
        ("solid.srs", po::value(&cfg.srs)->required()
         , "SRS definition. Use [WKT], +proj or EPSG:num.")
        ("solid.size", po::value(&cfg.size)->required()
         , "Size of dataset (WxH).")
        ("solid.extents", po::value(&cfg.extents)->required()
         , "Geo extents of dataset (ulx,uly:urx,ury).")
        ;

    try {
        std::ifstream f;
        f.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        f.open(openInfo->pszFilename);
        f.exceptions(std::ifstream::badbit);
        auto parsed(po::parse_config_file(f, config));
        po::store(parsed, vm);
        po::notify(vm);
        f.close();
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
        return new SolidDataset(cfg);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

SolidDataset::SolidDataset(const Config &config)
    : config_(config)
    , srs_(config.srs.as(geo::SrsDefinition::Type::wkt).srs)
    , tileSize_(256, 256)
    , overviews_(std::make_shared<RasterBands>())
{
    nRasterXSize = config_.size.width;
    nRasterYSize = config_.size.height;

    SetBand(1, new RasterBand(this, config_.size));

    auto size(config_.size);
    auto halve([&]()
    {
        size.width = int(std::round(size.width / 2.0));
        size.height = int(std::round(size.height / 2.0));
    });

    halve();
    while ((size.width >= tileSize_.width)
           || (size.height >= tileSize_.height))
    {
        overviews_->emplace_back(this, size);
        halve();
    }
}

CPLErr SolidDataset::GetGeoTransform(double *padfTransform)
{
    const auto &e(config_.extents);
    auto es(math::size(e));

    padfTransform[0] = e.ll(0);
    padfTransform[1] = es.width / nRasterXSize;
    padfTransform[2] = 0.0;

    padfTransform[3] = e.ur(1);
    padfTransform[4] = 0.0;
    padfTransform[5] = -es.height / nRasterYSize;;

    return CE_None;
}

const char* SolidDataset::GetProjectionRef()
{
    return srs_.c_str();
}

/* RasterBand */

SolidDataset::RasterBand::RasterBand(SolidDataset *dset
                                     , const math::Size2 &size)
    : size_(size)
{
    poDS = dset;
    nBand = 1;
    nBlockXSize = dset->tileSize_.width;
    nBlockYSize = dset->tileSize_.height;
    eDataType = dset->config_.dataType;

    nRasterXSize = size.width;
    nRasterYSize = size.height;
}

CPLErr SolidDataset::RasterBand::IReadBlock(int, int, void *rawImage)
{
    auto &dset(*static_cast<SolidDataset*>(poDS));
    switch (dset.config_.dataType) {
    case ::GDT_Byte:
        fill(static_cast<std::uint8_t*>(rawImage));
        break;

    case ::GDT_UInt16:
        fill(static_cast<std::uint16_t*>(rawImage));
        break;

    case ::GDT_Int16:
        fill(static_cast<std::int16_t*>(rawImage));
        break;

    case ::GDT_UInt32:
        fill(static_cast<std::uint32_t*>(rawImage));
        break;

    case ::GDT_Int32:
        fill(static_cast<std::int32_t*>(rawImage));
        break;

    case ::GDT_Float32:
        fill(static_cast<float*>(rawImage));
        break;

    case ::GDT_Float64:
        fill(static_cast<double*>(rawImage));
        break;

    default:
        // wtf?
        return CE_None;
    };

    return CE_None;
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

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
