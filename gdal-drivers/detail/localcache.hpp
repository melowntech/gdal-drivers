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
