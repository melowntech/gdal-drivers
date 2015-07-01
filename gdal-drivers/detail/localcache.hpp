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

    /** Tries to fetch tile from filesystem cache.
     *  Returns empty buffer on error.
     */
    Buffer fetchTile(const std::string &uri);

    /** Stores tile.
     */
    void storeTile(const std::string &uri, const Buffer &data);

private:
    const fs::path root_;
};

} } // namespace gdal_drivers::detail

#endif // gdal_drivers_detail_localcache_hpp_included_
