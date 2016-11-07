#ifndef gdal_drivers_detail_mbtiles_hpp_included_
#define gdal_drivers_detail_mbtiles_hpp_included_

#include "vector_tile.pb.h"

namespace gdal_drivers { namespace detail {

bool loadFromMbTilesArchive(vector_tile::Tile &tile, const char *path);

} } // namespace gdal_drivers::detail

#endif // gdal_drivers_detail_mbtiles_hpp_included_
