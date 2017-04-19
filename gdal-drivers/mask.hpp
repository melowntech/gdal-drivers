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
 * @file mask.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Quadtree Mask GDAL driver implementation
 *
 * Adds to GDAL an ability to work with mask dataset defined as a quadtree.
 */

#ifndef gdal_drivers_mask_hpp_included_
#define gdal_drivers_mask_hpp_included_

#include <gdal_priv.h>

#include <memory>
#include <vector>

#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "imgproc/rastermask/mappedqtree.hpp"
#include "geo/srsdef.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

/**
 * @brief GttDataset
 */

class MaskDataset : public GDALDataset {
public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~MaskDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

    /** Creates dataset from raster mask and extents.
     */
    static void create(const boost::filesystem::path &path
                       , const imgproc::quadtree::RasterMask &mask
                       , const math::Extents2 &extents
                       , const geo::SrsDefinition &srs
                       , unsigned int depth = 0
                       , unsigned int x = 0
                       , unsigned int y = 0);

private:
    MaskDataset(const fs::path &path, std::ifstream &f);

    class RasterBand;
    friend class RasterBand;

    typedef imgproc::mappedqtree::RasterMask Mask;
    Mask mask_;

    std::string srs_;
    math::Extents2 extents_;
    math::Size2 tileSize_;

    typedef std::vector<std::shared_ptr<RasterBand> > RasterBands;
    RasterBands overviews_;
};

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_MaskDataset(void);
CPL_C_END

#endif // gdal_drivers_mask_hpp_included_
