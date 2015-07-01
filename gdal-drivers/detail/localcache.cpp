#include <boost/crc.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/uri.hpp"
#include "utility/streams.hpp"
#include "utility/path.hpp"

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

} // namespace

Buffer LocalCache::fetchTile(const std::string &uri)
{
    const auto path(tilePath(root_, uri));

    try {
        std::ifstream f;
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(path.string(), std::ios_base::in);
        f.seekg(0, std::ifstream::end);
        auto size(f.tellg());
        f.seekg(0);
        Buffer data(size);
        f.read(&data[0], data.size());
        f.close();
        return data;
    } catch (const std::exception &e) {
        LOG(warn1) << "Cannot read tile <" << uri << "> from file "
                   << path  << ": " << e.what();
    }

    return {};
}

void LocalCache::storeTile(const std::string &uri, const Buffer &data)
{
    const auto path(tilePath(root_, uri));
    const auto tmpPath(utility::addExtension(path, ".tmp"));

    try {
        create_directories(tmpPath.parent_path());
        utility::write(path, data.data(), data.size());
        rename(tmpPath, path);
    } catch (const std::exception &e) {
        LOG(warn1) << "Cannot store tile <" << uri << "> in file "
                   << path  << ": " << e.what();
    }
}

} } // namespace gdal_drivers::detail
