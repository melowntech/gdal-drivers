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
#include <boost/crc.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/uri.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"
#include "utility/filesystem.hpp"
#include "utility/binaryio.hpp"
#include "utility/time.hpp"

#include "./localcache.hpp"

namespace gdal_drivers { namespace detail {

namespace {

std::uint32_t calculateHash(const std::string &data)
{
    boost::crc_32_type crc;
    crc.process_bytes(data.data(), data.size());
    return crc.checksum();
}

fs::path tilePath(const fs::path &root, const std::string &uri)
{
    auto fname(utility::urlEncode(uri));
    auto hash(calculateHash(fname));
    return root
        / str(boost::format("%02x/%02x")
              % ((hash >> 24) & 0xff)
              % ((hash >> 16) & 0xff)
              )
        / fname;
}

namespace bio = utility::binaryio;

void removeTile(const fs::path &path)
{
    boost::system::error_code ec;
    fs::remove(path, ec);
}

} // namespace

LocalCache::Tile LocalCache::fetchTile(const std::string &uri)
{
    const auto path(tilePath(root_, uri));
    LOG(debug) << "Loading tile <" << uri << "> from file "
               << path  << ".";

    try {
        utility::ifstreambuf f(path.string());
        f.seekg(0, std::ifstream::end);
        std::size_t size(f.tellg());
        f.seekg(0);

        Tile tile(Tile::Type::notFound);

        // read type field
        std::uint8_t type;
        bio::read(f, type);
        switch (type) {
        case 0: tile.type = Tile::Type::empty; break;
        case 1: tile.type = Tile::Type::valid; break;
        default:
            // invalid data
            tile.type = Tile::Type::notFound;
            removeTile(path);
            return tile;
        }

        // read expires field
        std::int64_t expires;
        bio::read(f, expires);
        tile.expires = expires;

        // check for expiration
        if (tile.expires
            && (std::time_t(utility::currentTime().first) > tile.expires)) {
            // invalid data
            tile.type = Tile::Type::notFound;
            removeTile(path);
            return tile;
        }

        if (tile.type == Tile::Type::valid) {
            tile.data.resize(size - sizeof(type) - sizeof(expires));
            bio::read(f, tile.data.data(), tile.data.size());
        }
        f.close();
        return tile;
    } catch (const std::exception &e) {
        LOG(debug) << "Cannot read tile <" << uri << "> from file "
                   << path  << ": " << e.what() << ".";
        removeTile(path);
    }

    return { Tile::Type::notFound };
}

void LocalCache::storeTile(const std::string &uri, const Tile &tile)
{
    std::uint8_t type(0);
    switch (tile.type) {
    case Tile::Type::empty: type = 0; break;
    case Tile::Type::valid: type = 1; break;
    default:
        // ignore anything else unknown
        return;
    }

    const auto path(tilePath(root_, uri));
    const auto tmpPath(utility::addExtension(path, ".tmp"));

    LOG(debug) << "Storing tile <" << uri << "> into file "
               << path  << ".";

    try {
        create_directories(tmpPath.parent_path());

        {
            utility::ofstreambuf f(tmpPath.string());
            bio::write(f, type);
            std::int64_t expires(tile.expires);
            bio::write(f, expires);
            if (tile.type == Tile::Type::valid) {
                bio::write(f, tile.data.data(), tile.data.size());
            }
            f.close();
        }

        rename(tmpPath, path);
    } catch (const std::exception &e) {
        LOG(debug) << "Cannot store tile <" << uri << "> in file "
                   << path  << ": " << e.what() << ".";
    }
}

} } // namespace gdal_drivers::detail
