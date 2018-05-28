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
#include "imgproc/rastermask/quadtree.hpp"
#include "imgproc/fillrect.hpp"

#include "./mask.hpp"

namespace fs = boost::filesystem;
namespace bin = utility::binaryio;

namespace gdal_drivers {

namespace {
    const char IO_MAGIC[6] = { 'G', 'D', 'A', 'L', 'Q', 'M' };
} // namespace

/**
 * @brief BorderedAreaRasterBand
 */

class MaskDataset::RasterBand : public GDALRasterBand {
public:
    RasterBand(MaskDataset *dset, unsigned int depth);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~RasterBand() {};

    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 1; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return GCI_GrayIndex; }

    virtual int GetOverviewCount() {
        const auto &overviews(static_cast<MaskDataset*>(poDS)->overviews_);
        return overviews.size();
    }

    virtual GDALRasterBand* GetOverview(int index) {
        const auto &overviews(static_cast<MaskDataset*>(poDS)->overviews_);
        if (index >= int(overviews.size())) { return nullptr; }
        return overviews[index].get();
    }

private:
    unsigned int depth_;
    unsigned int tail_;
    cv::Rect tileBounds_;
};

GDALDataset* MaskDataset::Open(GDALOpenInfo *openInfo)
{
    ::CPLErrorReset();

    // try to open
    utility::ifstreambuf f;
    try {
        f.open(openInfo->pszFilename);
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

    SetBand(1, new RasterBand(this, mask_.depth()));

    auto depth(mask_.depth());
    while (depth) {
        --depth;
        overviews_.push_back(std::make_shared<RasterBand>(this, depth));
    }
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

MaskDataset::RasterBand::RasterBand(MaskDataset *dset, unsigned int depth)
    : depth_(depth), tail_(dset->mask_.depth() - depth_)
    , tileBounds_(0, 0, dset->tileSize_.width, dset->tileSize_.height)
{
    poDS = dset;
    nBand = 1;
    nBlockXSize = dset->tileSize_.width;
    nBlockYSize = dset->tileSize_.height;
    eDataType = GDT_Byte;

    nRasterXSize = dset->mask_.size().width >> tail_;
    nRasterYSize = dset->mask_.size().height >> tail_;
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
    int xShift(blockCol * ts.width);
    int yShift(blockRow * ts.height);

    // setup constraints
    Mask::Constraints con(depth_);
    con.extents.ll(0) = (xShift << tail_);
    con.extents.ll(1) = (yShift << tail_);
    con.extents.ur(0) = con.extents.ll(0) + (ts.width << tail_);
    con.extents.ur(1) = con.extents.ll(1) + (ts.height << tail_);

    try {
        // wrap tile into matrix and reset to to zero
        cv::Mat tile(ts.height, ts.width, CV_8UC1, rawImage);
        tile = cv::Scalar(color::black);

        auto draw([&](Mask::Node node, boost::tribool value)
        {
            // black -> nothing
            if (!value) { return; }

            // update to match level grid
            node.shift(tail_);

            node.x -= xShift;
            node.y -= yShift;

            // construct rectangle and intersect it with bounds
            cv::Rect r(node.x, node.y, node.size, node.size);
            auto rr(r & tileBounds_);
            imgproc::fillRectangle
                (tile, rr, (value ? color::white : color::gray));
        });

        dset.mask_.forEachQuad(draw, con);
    } catch (const std::exception &e) {
        CPLError(CE_Failure, CPLE_FileIO, "%s\n", e.what());
        return CE_Failure;
    }
    return CE_None;
}

void MaskDataset::create(const boost::filesystem::path &path
                         , const imgproc::quadtree::RasterMask &mask
                         , const math::Extents2 &extents
                         , const geo::SrsDefinition &srs
                         , unsigned int depth, unsigned int x, unsigned int y)
{
    utility::ofstreambuf f(path.string());

    bin::write(f, IO_MAGIC); // 6 bytes
    bin::write(f, uint8_t(0)); // reserved
    bin::write(f, uint8_t(0)); // reserved

    // write SRS
    {
        auto srsWkt(srs.as(geo::SrsDefinition::Type::wkt).srs);
        bin::write(f, std::uint32_t(srsWkt.size()));
        bin::write(f, srsWkt.data(), srsWkt.size());
    }

    // update extents to to square
    auto sExtents(extents);
    {
        // input raster size
        auto mSize(mask.size());
        // square raster size
        auto sSize(1 << mask.depth());
        // extents size
        auto es(math::size(extents));
        // update extents to be square
        sExtents.ur(0) = sExtents.ll(0) + ((es.width * sSize) / mSize.width);
        sExtents.ll(1) = sExtents.ur(1) - ((es.height * sSize) / mSize.height);
    }

    // write extents
    bin::write(f, double(sExtents.ll(0)));
    bin::write(f, double(sExtents.ll(1)));
    bin::write(f, double(sExtents.ur(0)));
    bin::write(f, double(sExtents.ur(1)));

    // write mask
    imgproc::mappedqtree::RasterMask::write(f, mask, depth, x, y);
    f.close();
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
