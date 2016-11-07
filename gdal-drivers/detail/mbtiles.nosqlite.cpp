#include <cpl_error.h>

#include "./mbtiles.hpp"

namespace gdal_drivers { namespace detail {

bool loadFromMbTilesArchive(vector_tile::Tile&, const char*)
{
    ::CPLError(CE_Failure, CPLE_NotSupported
               , "mbtiles unsupported: gdal-drivers compiled "
               "without Sqlite3 support");
    return false;
}

} } // namespace gdal_drivers::detail
