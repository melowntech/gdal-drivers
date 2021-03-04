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
/** mvt.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Virtual GDAL driver that returns mvt valid in whole extents.
 */

#ifndef gdal_drivers_mvt_hpp_included_
#define gdal_drivers_mvt_hpp_included_

#include <gdal_priv.h>

#include <memory>
#include <array>
#include <vector>

#include <boost/variant.hpp>
#include <boost/filesystem/path.hpp>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"
#include "geo/geotransform.hpp"

#include "vector_tile.pb.h"

namespace gdal_drivers {

/**
 * @brief GttDataset
 */

class MvtDataset : public GDALDataset {
public:
    static ::GDALDataset* Open(::GDALOpenInfo *openInfo);
    static int Identify(::GDALOpenInfo *openInfo);

    class Layer;
    friend class Layer;

    virtual int GetLayerCount() override { return layers_.size(); }

    virtual OGRLayer* GetLayer(int) override;
    virtual OGRLayer* GetLayerByName(const char *name) override;

private:
    MvtDataset(std::unique_ptr<vector_tile::Tile> tile
               , const boost::optional<geo::SrsDefinition> &srs
               , const boost::optional<math::Extents2> &extents
               , bool noFields);

    std::unique_ptr<vector_tile::Tile> tile_;
    boost::optional<geo::SrsDefinition> srs_;
    boost::optional<math::Extents2> extents_;
    bool noFields_;
    std::vector<std::unique_ptr<Layer>> layers_;
};

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_MvtDataset(void);
CPL_C_END

#endif // gdal_drivers_mvt_hpp_included_
