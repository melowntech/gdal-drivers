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
/**
 * @file mapy-cz.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Bordered Aread GDAL driver implementation
 *
 * Adds to GDAL an ability to work with special mask data set:
 *     * mask is stored in GeoTiff
 *     * each pixel represents 256x256 tile
 *     * tile inside border is full white
 *     * tile outside border is full black
 *     * tile exactly on border is stored in its one PNG image
 */

#ifndef gdal_drivers_borderedarea_hpp_included_
#define gdal_drivers_borderedarea_hpp_included_

#include <gdal_priv.h>

#include <map>

#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include <curl/curl.h>

#include "math/geometry_core.hpp"

#include "geo/geodataset.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

class BorderedAreaRasterBand;

/**
 * @brief GttDataset
 */

class BorderedAreaDataset : public GDALDataset {
public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~BorderedAreaDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

    friend class BorderedAreaRasterBand;

    static void asMask(const fs::path &srcPath, const fs::path &dstPath);

private:
    BorderedAreaDataset(const fs::path &root);

    const cv::Mat& getTile(const math::Point2i &tile);

    const fs::path root_;
    geo::GeoDataset mask_;
    std::string srs_;
    math::Size2 tileSize_;

    // List tile read
    math::Point2i lastTile_;
    cv::Mat lastTileImage_;

    // last block read from underlying GeoTiff
    geo::GeoDataset::Block lastBlock_;
    math::Point2i lastBlockOffset_;

    /** Tile full of black pixels
     */
    cv::Mat blackTile_;

    /** Tile full of white pixels
     */
    cv::Mat whiteTile_;
};

/**
 * @brief BorderedAreaRasterBand
 */

class BorderedAreaRasterBand : public GDALRasterBand
{

public:
    BorderedAreaRasterBand(BorderedAreaDataset *dset);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~BorderedAreaRasterBand() {};

    /** 0 is special marker for no-data pixels
     */
    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 1; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return GCI_GrayIndex; }

private:
    int cvChannel_;
    GDALColorInterp colorInterp_;
};

} // namespace gdal_drivers


// driver registration function
CPL_C_START
void GDALRegister_BorderedArea(void);
CPL_C_END

#endif // gdal_drivers_borderedarea_hpp_included_
