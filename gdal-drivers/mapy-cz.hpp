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
 * Mapy.cz GDAL driver implementation
 *
 * Adds to GDAL an ability to work with map data from mapy.cz.
 * Dataset uri: mapycz://MAP_TYPE/ZOOM where:
 *    * MAP_TYPE is one of base, ortho
 *    * ZOOM is a level-of-detail specifier in range 0-20
 */

#ifndef gdal_drivers_mapy_cz_hpp_included_
#define gdal_drivers_mapy_cz_hpp_included_

#include <gdal_priv.h>

#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include "math/geometry_core.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

class MapyczRasterBand;

namespace detail {
    class Fetcher;
} // namespace detail

/**
 * @brief GttDataset
 */

class MapyczDataset : public GDALDataset {

    friend class MapyczRasterBand;

public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~MapyczDataset();

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

private:
    MapyczDataset(const std::string &mapType, int zoom, unsigned int flags);

    const cv::Mat& getTile(const math::Point2i &tile);

    const std::string mapType_;
    int zoom_;

    std::string srs_;

    /** Pixel size in meters.
     */
    math::Size2f pixelSize_;

    /** Tile size in PP units.
     */
    math::Size2_<long> tileSize_;

    math::Point2i lastTile_;
    cv::Mat lastTileImage_;

    enum Flag : unsigned int { none = 0x0 };

    unsigned int flags_;

    std::unique_ptr<detail::Fetcher> fetcher_;
};

/**
 * @brief MapyczRasterBand
 */

class MapyczRasterBand : public GDALRasterBand
{

public:
    MapyczRasterBand(MapyczDataset *dset, int numBand
                     , int cvChannel
                     , GDALColorInterp colorInterp);

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~MapyczRasterBand() {};

    /** 0 is special marker for no-data pixels
     * NB: all valid pixels with 0 in original data are changed to 1 internally
     */
    virtual double GetNoDataValue(int *success = nullptr) {
        if (success) { *success = 1; }
        return 0.0;
    }

    virtual GDALColorInterp GetColorInterpretation() { return colorInterp_; }

private:
    int cvChannel_;
    GDALColorInterp colorInterp_;
};

} // namespace gdal_drivers


// driver registration function
CPL_C_START
void GDALRegister_MapyCz(void);
CPL_C_END

#endif // gdal_drivers_mapy_cz_hpp_included_
