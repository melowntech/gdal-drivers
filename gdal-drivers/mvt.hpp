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

    class Layer;
    friend class Layer;

    virtual int GetLayerCount() { return layers_.size(); }

    virtual OGRLayer* GetLayer(int);
    virtual OGRLayer* GetLayerByName(const char *name);

private:
    MvtDataset(std::unique_ptr<vector_tile::Tile> tile
               , const boost::optional<geo::SrsDefinition> &srs
               , const boost::optional<math::Extents2> &extents);

    std::unique_ptr<vector_tile::Tile> tile_;
    boost::optional<geo::SrsDefinition> srs_;
    boost::optional<math::Extents2> extents_;
    std::vector<std::unique_ptr<Layer>> layers_;
};

} // namespace gdal_drivers

// driver registration function
CPL_C_START
void GDALRegister_MvtDataset(void);
CPL_C_END

#endif // gdal_drivers_mvt_hpp_included_
