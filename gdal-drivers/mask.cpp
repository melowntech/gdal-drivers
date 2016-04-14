/*
 * @file mask.cpp
 */

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>

#include <boost/filesystem/path.hpp>
#include <boost/logic/tribool_io.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/streams.hpp"
#include "utility/binaryio.hpp"

#include "./mask.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace gdal_drivers {

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

private:
    cv::Rect tileBounds_;
};

GDALDataset* MaskDataset::Open(GDALOpenInfo *openInfo)
{
    ::CPLErrorReset();

    // try to open
    utility::ifstreambuf f;
    try {
        f.open(openInfo->pszFilename);
        const char IO_MAGIC[6] = { 'G', 'D', 'A', 'L', 'Q', 'M' };
        char magic[6];
        bin::read(f, magic);
        if (std::memcmp(magic, IO_MAGIC, sizeof(IO_MAGIC))) {
            return nullptr;
        }
    } catch (...) { return nullptr; }

    // file belongs to us, we can continue

    // no updates
    if (openInfo->eAccess == GA_Update) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "The Quadtree Mask driver does not support update "
                 "access to existing datasets.\n");
        return nullptr;
    }

    // initialize dataset
    try {
        return new MaskDataset(openInfo->pszFilename, f);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

MaskDataset::MaskDataset(const fs::path &path, std::ifstream &f)
    : tileSize_(256, 256)
{
    auto maskOffset([&]() -> std::size_t
    {
        // load reserved
        {
            std::uint8_t reserved;
            bin::read(f, reserved);
            bin::read(f, reserved);
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
    auto es(math::size(extents_));

    padfTransform[0] = extents_.ll(0);
    padfTransform[1] = es.width / nRasterXSize;
    padfTransform[2] = 0.0;

    padfTransform[3] = extents_.ur(1);
    padfTransform[4] = 0.0;
    padfTransform[5] = -es.height / nRasterYSize;;

    return CE_None;
}

const char* MaskDataset::GetProjectionRef()
{
    return srs_.c_str();
}

/* RasterBand */

MaskDataset::RasterBand::RasterBand(MaskDataset *dset)
    : tileBounds_(0, 0, dset->tileSize_.width, dset->tileSize_.height)
{
    poDS = dset;
    nBand = 1;
    nBlockXSize = dset->tileSize_.width;
    nBlockYSize = dset->tileSize_.height;
    eDataType = GDT_Byte;
}

namespace color {
    cv::Scalar black(0x00);
    cv::Scalar white(0xff);
    cv::Scalar gray(0x80);
} // namespace color

CPLErr MaskDataset::RasterBand::IReadBlock(int blockCol, int blockRow
                                           , void *rawImage)
{
    const auto &dset(*static_cast<MaskDataset*>(poDS));

    const auto &ts(dset.tileSize_);
    unsigned int xShift(blockCol * ts.width);
    unsigned int yShift(blockRow * ts.height);

    try {
        // wrap tile into matrix and reset to to zero
        cv::Mat tile(ts.height, ts.width, CV_8UC1, rawImage);
        tile = cv::Scalar(color::black);

        auto draw([&](unsigned int x, unsigned int y
                      , unsigned int size, boost::tribool value)
        {
            // black -> nothing
            if (!value) { return; }

            x -= xShift;
            y -= yShift;

            // construct rectangle and intersect it with bounds
            cv::Rect r(x, y, size, size);
            auto rr(r & tileBounds_);

            // draw white or gray rectangle
            cv::rectangle(tile, rr
                          , (value ? color::white : color::gray)
                          , CV_FILLED, 4);
        });

        dset.mask_.forEachQuad(draw);
    } catch (const std::exception &e) {
        CPLError(CE_Failure, CPLE_FileIO, "%s\n", e.what());
        return CE_Failure;
    }
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
