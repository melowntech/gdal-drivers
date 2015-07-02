/**
 * @file localcache.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Local filesystem cache.
 */

#ifndef gdal_drivers_detail_localcache_hpp_included_
#define gdal_drivers_detail_localcache_hpp_included_

#include <vector>

#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

namespace gdal_drivers { namespace detail {

typedef std::vector<char> Buffer;

class LocalCache {
public:
    LocalCache(const fs::path &root) : root_(root) {}

    struct Tile {
        enum class Type {
            empty, valid, notFound
        };

        Type type;
        Buffer data;
        std::time_t expires;

        Tile(Type type = Type::valid) : type(type), expires() {}
    };

    /** Tries to fetch tile from filesystem cache.
     */
    Tile fetchTile(const std::string &uri);

    /** Stores tile.
     */
    void storeTile(const std::string &uri, const Tile &tile);

private:
    const fs::path root_;
};

} } // namespace gdal_drivers::detail

#endif // gdal_drivers_detail_localcache_hpp_included_
