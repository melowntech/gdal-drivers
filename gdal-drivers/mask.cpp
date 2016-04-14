/*
 * @file mask.cpp
 */

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>

#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/binaryio.hpp"

#include "./mask.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace gdal_drivers {

namespace def {
    const fs::path MaskPath("borderedarea.tif");
    const fs::path HelperPath("borderedarea");
} // namespace def

/**
 * @brief BorderedAreaRasterBand
 */

class MaskDataset::RasterBand : public GDALRasterBand {
public:
    RasterBand(MaskDataset *dset);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~RasterBand() {};

    /** 0 is special marker for no-data pixels
     */
    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 1; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return GCI_GrayIndex; }
};

GDALDataset* MaskDataset::Open(GDALOpenInfo *openInfo)
{
    // no updates
    if (openInfo->eAccess == GA_Update) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "The Quadtree Mask driver does not support update "
                 "access to existing datasets.\n");
        return nullptr;
    }

    // initialize dataset
    try {
        return new MaskDataset(openInfo->pszFilename);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

MaskDataset::MaskDataset(const fs::path &path)
    : tileSize_(256, 256)
{
    auto maskOffset([&]() -> std::size_t
    {
        utility::ifstreambuf f(path.string());

        const char IO_MAGIC[6] = { 'G', 'D', 'A', 'L', 'Q', 'M' };;

        char magic[6];
        bin::read(f, magic);
        if (std::memcmp(magic, IO_MAGIC, sizeof(IO_MAGIC))) {
            throw std::runtime_error("Not a GDAL Quadtree Mask.");
        }

        {
            // load srs
            std::uint32_t size;
            bin::read(f, size);
            std::vector<char> tmp(size, 0);
            bin::read(f, tmp.data(), tmp.size());
            srs_.assign(tmp.data(), tmp.size());
        }

        // read extents
        bin::read(f, extents_.ll(0));
        bin::read(f, extents_.ll(1));
        bin::read(f, extents_.ur(0));
        bin::read(f, extents_.ur(1));

        auto end(f.tellg());
        f.close();
        return end;
    }());

    // load mask
    mask_ = Mask(path, maskOffset);

    nRasterXSize = mask_.size().width;
    nRasterYSize = mask_.size().height;

    SetBand(1, new RasterBand(this));
}

CPLErr MaskDataset::GetGeoTransform(double *padfTransform)
{
#if 0
    const auto extents(mask_.extents());
    const auto resolution(mask_.resolution());

    padfTransform[0] = extents.ll(0);
    padfTransform[1] = resolution(0) / double(tileSize_.width);
    padfTransform[2] = 0.0;

    padfTransform[3] = extents.ur(1);
    padfTransform[4] = 0.0;
    padfTransform[5] = -resolution(1) / double(tileSize_.height);
#endif
    (void) padfTransform;
    return CE_None;
}

const char* MaskDataset::GetProjectionRef()
{
    return srs_.c_str();
}

/* RasterBand */

MaskDataset::RasterBand::RasterBand(MaskDataset *dset)
{
    poDS = dset;
    nBand = 1;
    nBlockXSize = dset->tileSize_.width;
    nBlockYSize = dset->tileSize_.height;
    eDataType = GDT_Byte;
}

CPLErr MaskDataset::RasterBand::IReadBlock(int blockCol, int blockRow
                                           , void *rawImage)
{
    (void) blockCol;
    (void) blockRow;
    (void) rawImage;

#if 0
    auto &dset(*static_cast<MaskDataset*>(poDS));

    try {
        cv::Mat tile(dset.getTile({blockCol, blockRow}));
        cv::Mat image(dset.tileSize_.height, dset.tileSize_.width
                      , CV_8UC1, rawImage);
        tile.copyTo(image);
    } catch (const std::exception &e) {
        CPLError(CE_Failure, CPLE_FileIO, "%s\n", e.what());
        return CE_Failure;
    }
#endif
    return CE_None;
}

} // namespace gdal_drivers

/* GDALRegister_MaskDataset */

void GDALRegister_MaskDataset()
{
    if (!GDALGetDriverByName("QuadtreeMask")) {
        std::unique_ptr<GDALDriver> driver(new GDALDriver());

        driver->SetDescription("QuadtreeMask");
        driver->SetMetadataItem
            (GDAL_DMD_LONGNAME
             , "Support for mask defined as quadtree mask.");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        driver->pfnOpen = gdal_drivers::MaskDataset::Open;

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
