#include <boost/utility/in_place_factory.hpp>

#include "dbglog/dbglog.hpp"

#include "utility/uri.hpp"
#include "utility/time.hpp"

#include "imgproc/readimage.hpp"

#include "./fetcher.hpp"

extern "C" {

size_t gdal_drivers_detail_fetcher_write(char *ptr, size_t size, size_t nmemb
                                         , void *userdata)
{
    auto &buffer(*static_cast<gdal_drivers::detail::Buffer*>(userdata));
    auto bytes(size * nmemb);
    std::copy(ptr, ptr + bytes, std::back_inserter(buffer));
    return bytes;
}

} // extern "C"

namespace gdal_drivers { namespace detail {

namespace {

std::shared_ptr< ::CURL> createCurl()
{
    auto c(::curl_easy_init());
    if (!c) {
        LOGTHROW(err2, std::runtime_error) << "Failed to create CURL handle.";
    }

    // switch off SIGALARM
    ::curl_easy_setopt(c, CURLOPT_NOSIGNAL, 1);

    return std::shared_ptr< ::CURL>
        (c, [](CURL *c) { if (c) { ::curl_easy_cleanup(c); } });
}

cv::Mat decodeImage(const std::string &url, const math::Size2i &tileSize
                    , const Buffer &buffer, bool skipCorrupted)
{
    auto image(imgproc::readImage(buffer.data(), buffer.size()));
    if (!image.data) {
        if (skipCorrupted) {
            LOG(warn2) << "Failed to decode tile data downloaded from <"
                       << url << ">, using black image instead.";
            image = cv::Mat( tileSize.height, tileSize.width
                           , CV_8UC3, cv::Scalar(1, 1, 1));
        } else {
            LOGTHROW(err2, std::runtime_error)
                << "Failed to decode tile data downloaded from <"
                << url << ">.";
        }
    }

    if ((image.cols != tileSize.width)
        || (image.rows != tileSize.height))
    {
        LOGTHROW(err2, std::runtime_error)
            << "Tile downloaded from from <"
            << url << "> has wrong dimensions: "
            << image.cols << "x" << image.rows << " (should be "
            << tileSize << ").";
    }

    // convert 0 to 1 to prevent interpretation as no-data value
    {
        auto src(static_cast<unsigned char*>(image.data));
        std::size_t width(image.cols * image.channels());
        for (std::size_t y(0); y < std::size_t(image.rows); ++y) {
            auto *sdata(src + image.step * y);
            for (std::size_t x(0); x < width; ++x, ++sdata) {
                if (!*sdata) { *sdata = 1; }
            }
        }
    }

    return image;
}

#define CHECK_CURL_STATUS(what)                                         \
    do {                                                                \
        auto res(what);                                                 \
        if (res != CURLE_OK) {                                          \
            LOGTHROW(err2, std::runtime_error)                          \
                << "Failed to download tile from <"                     \
                << url << ">: <" << res << ", "                         \
                << ::curl_easy_strerror(res)                            \
                << ">.";                                                \
        }                                                               \
    } while (0)

long int fetchUrl(::CURL *curl, const std::string &url
                  , Buffer &buffer)
{
    LOG(info1) << "Fetching tile from <" << url << "> "
               << "(GET).";

    buffer.clear();

    // we are getting a resource
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_HTTPGET, 1L));
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_NOBODY, 0L));

    // HTTP/1.1
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_HTTP_VERSION
                                         , CURL_HTTP_VERSION_1_1));
    // target url
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_URL, url.c_str()));

    // do not follow redirects -> we can detect non-existent tiles
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 0L));

    auto uri(utility::parseUri(url));
    if (!uri.host.empty()) {
        // set referer :)
        uri.search = uri.user = uri.password = uri.path = "";
        CHECK_CURL_STATUS(::curl_easy_setopt
                          (curl, CURLOPT_REFERER, uri.join().c_str()));
    } else {
        CHECK_CURL_STATUS(::curl_easy_setopt
                          (curl, CURLOPT_REFERER, nullptr));
    }

    // set output function + userdata
    CHECK_CURL_STATUS(::curl_easy_setopt
                      (curl, CURLOPT_WRITEFUNCTION
                       , &gdal_drivers_detail_fetcher_write));
    CHECK_CURL_STATUS(::curl_easy_setopt(curl, CURLOPT_WRITEDATA, &buffer));

    /// do the thing
    CHECK_CURL_STATUS(::curl_easy_perform(curl));

    // check status code:
    long int httpCode(0);
    CHECK_CURL_STATUS(::curl_easy_getinfo
                      (curl, CURLINFO_RESPONSE_CODE, &httpCode));

    return httpCode;
}

#undef CHECK_CURL_STATUS

cv::Mat blackTile(const math::Size2i &tileSize)
{
    cv::Mat black(tileSize.height, tileSize.width, CV_8UC(3)
                  , cv::Scalar(0, 0, 0));
    return black;
}

cv::Mat fetchTile(LocalCache *cache, ::CURL *curl
                  , const math::Size2i &tileSize
                  , const std::string &url
                  , bool skipCorrupted)
{
    LocalCache::Tile tile;
    if (cache) {
        tile = cache->fetchTile(url);

        switch (tile.type) {
        case LocalCache::Tile::Type::empty:
            return blackTile(tileSize);

        case LocalCache::Tile::Type::valid:
            // tile loaded from cache
            try {
                auto image(decodeImage(url, tileSize, tile.data, false));
                LOG(info1) << "Tile from <" << url << "> fetched (cached).";
                return image;
            } catch (std::exception&) {
                // cannot decode
            }

        default: break; // not found
        }
    }

    // fetch from web

    auto httpCode(fetchUrl(curl, url, tile.data));

    if (httpCode == 302) {
        // TODO: check redirect location
        // no imagery for this tile -> return black one
        LOG(info1) << "Tile from <" << url << "> fetched.";

        if (cache) {
            tile.type = LocalCache::Tile::Type::empty;
            tile.expires = utility::currentTime().first + 604800; // a week
            cache->storeTile(url, tile);
        }

        return blackTile(tileSize);
    }

    if (httpCode != 200) {
        LOGTHROW(err2, std::runtime_error)
            << "Failed to download tile data from <"
            << url << ">: Unexpected HTTP status code: <" << httpCode << ">.";
    }

    auto image(decodeImage(url, tileSize, tile.data, skipCorrupted));

    if (cache) {
        tile.type = LocalCache::Tile::Type::valid;
        tile.expires = utility::currentTime().first + 604800; // a week
        cache->storeTile(url, tile);
    }

    LOG(info1) << "Tile from <" << url << "> fetched.";
    return image;
}

cv::Mat fetchTileSafe(LocalCache *cache, ::CURL *curl
                      , const math::Size2i &tileSize
                      , const std::string &url
                      , unsigned int tries)
{
    for (; tries; --tries) {
        try {
            return fetchTile(cache, curl, tileSize, url, false);
        } catch (const std::exception &e) {
            LOG(warn2) << "Failed to fetch tile, retrying.";
            ::sleep(1);
        }
    }

    // final try, let's it fall through on failure
    return fetchTile(cache, curl, tileSize, url, true);
}

} // namespace

Fetcher::Fetcher(const math::Size2i &tileSize
                 , const boost::optional<std::string> &proxy
                 , const boost::optional<fs::path> &cacheRoot)
    : tileSize_(tileSize), curl_(createCurl())
{
    if (cacheRoot) {
        cache_ = boost::in_place(*cacheRoot);
    }

    if (proxy) {
        // set proxy
        ::curl_easy_setopt(curl_.get(), CURLOPT_PROXY, proxy->c_str());
    }

}

cv::Mat Fetcher::getTile(const std::string &url)
{
    return fetchTileSafe(cache_.get_ptr(), curl_.get()
                         , tileSize_, url, 20);
}

} } // namespace gdal_drivers::detail
