/**
 * @file fetcher.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Tile fetcher.
 */

#ifndef gdal_drivers_detail_fetcher_hpp_included_
#define gdal_drivers_detail_fetcher_hpp_included_

#include <memory>

#include <boost/optional.hpp>

#include <opencv2/core/core.hpp>

#include <curl/curl.h>

#include "math/geometry_core.hpp"

#include "./localcache.hpp"

namespace gdal_drivers { namespace detail {

class Fetcher {
public:
    Fetcher(const math::Size2i &tileSize
            , const boost::optional<std::string> &proxy
            , const boost::optional<fs::path> &cacheRoot);

    cv::Mat getTile(const std::string &url);

private:
    const math::Size2i tileSize_;
    boost::optional<LocalCache> cache_;

    std::shared_ptr< ::CURL> curl_;
};

} } // namespace gdal_drivers::detail

#endif // gdal_drivers_detail_fetcher_hpp_included_
