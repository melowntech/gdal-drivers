/*
 * mapy-cz.cpp
 */

#include <cstdlib>
#include <algorithm>
#include <vector>
#include <iterator>

#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "dbglog/dbglog.hpp"
#include "utility/uri.hpp"
#include "utility/path.hpp"
#include "geo/srsdef.hpp"
#include "imgproc/readimage.hpp"

#include "./borderedarea.hpp"

namespace gdal_drivers {

namespace def {
    const fs::path MaskPath("borderedarea.tif");
} // namespace def

/* class BorderedAreaDataset */

GDALDataset* BorderedAreaDataset::Open(GDALOpenInfo *openInfo)
{
    const auto path(openInfo->pszFilename / def::MaskPath);

    if (!exists(path)) { return nullptr; }

    // no updates
    if (openInfo->eAccess == GA_Update) {
        CPLError(CE_Failure, CPLE_NotSupported,
                 "The BorderedAread driver does not support update "
                 "access to existing datasets.\n");
        return nullptr;
    }

    // initialize dataset
    try {
        return new BorderedAreaDataset(openInfo->pszFilename);
    } catch (const std::runtime_error & e) {
        CPLError(CE_Failure, CPLE_IllegalArg
                 , "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }
}

namespace {

math::Size2 getTileSize(const geo::GeoDataset &mask)
{
    auto metadata(mask.getMetadata("BORDEREDAREA"));
    auto width(metadata.get<int>("TILE_WIDTH"));
    auto height(metadata.get<int>("TILE_HEIGHT"));
    if (!width || !height) {
        LOGTHROW(err2, std::runtime_error)
            << "Invalid underlying GeoTiff: missing tile size.";
    }
    return { *width, *height };
}

} // namespace

BorderedAreaDataset::BorderedAreaDataset(const fs::path &root)
    : root_(root)
    , mask_(geo::GeoDataset::createFromFS(root / def::MaskPath))
    , srs_(mask_.srsWkt())
    , tileSize_(getTileSize(mask_))
    , blackTile_(tileSize_.height, tileSize_.width, CV_8UC1)
    , whiteTile_(tileSize_.height, tileSize_.width, CV_8UC1)
{
    // underlying tiff must be gray
    mask_.expectGray();

    nRasterXSize = (tileSize_.width * mask_.size().width);
    nRasterYSize = (tileSize_.height * mask_.size().height);

    SetBand(1, new BorderedAreaRasterBand(this));

    blackTile_ = cv::Scalar(0x00);
    whiteTile_ = cv::Scalar(0xff);
}

CPLErr BorderedAreaDataset::GetGeoTransform(double *padfTransform)
{
    const auto extents(mask_.extents());
    const auto resolution(mask_.resolution());

    padfTransform[0] = extents.ll(0);
    padfTransform[1] = resolution(0) / double(tileSize_.width);
    padfTransform[2] = 0.0;

    padfTransform[3] = extents.ur(1);
    padfTransform[4] = 0.0;
    padfTransform[5] = -resolution(1) / double(tileSize_.height);

    return CE_None;
}

const char* BorderedAreaDataset::GetProjectionRef()
{
    return srs_.c_str();
}

const cv::Mat& BorderedAreaDataset::getTile(const math::Point2i &tile)
{
    if (lastTileImage_.data && (tile == lastTile_)) {
        // tile cached
        return lastTileImage_;
    }

    math::Point2i blockOffset;
    math::Point2i pixelIndex;
    std::tie(blockOffset, pixelIndex) = mask_.blockCoord(tile);

    if (!lastBlock_ || (blockOffset != lastBlockOffset_)) {
        // we need to load new block
        lastBlock_ = mask_.readBlock(blockOffset);
        lastBlockOffset_ = blockOffset;
    }

    auto color(lastBlock_.data.at<double>(pixelIndex(1), pixelIndex(0)));

    if (color == 0x00) {
        // masked-out
        return blackTile_;
    } else if (color == 0xff) {
        // valid
        return whiteTile_;
    }

    // detailed mask available, load tile from file
    auto path(root_ / str(boost::format("%06d/%06d.png")
                          % tile(1) % tile(0)));
    auto image(cv::imread(path.string(), CV_LOAD_IMAGE_GRAYSCALE));
    if (!image.data) {
        LOGTHROW(err2, std::runtime_error)
            << "Missing imagery for tile " << path << ".";
    }

    if ((image.cols != tileSize_.width) || (image.rows != tileSize_.height)) {
        LOGTHROW(err2, std::runtime_error)
            << "Imagery for tile " << path << " has dimensions different "
            "from tileSize.";
    }

    lastTileImage_ = image;
    lastTile_ = tile;

    // done
    return lastTileImage_;
}

/* BorderedAreaRasterBand */

BorderedAreaRasterBand::BorderedAreaRasterBand(BorderedAreaDataset *dset)
{
    poDS = dset;
    nBand = 1;
    nBlockXSize = dset->tileSize_.width;
    nBlockYSize = dset->tileSize_.height;
    eDataType = GDT_Byte;
}

CPLErr BorderedAreaRasterBand::IReadBlock(int blockCol, int blockRow
                                    , void *rawImage)
{
    auto &dset(*static_cast<BorderedAreaDataset*>(poDS));

    try {
        cv::Mat tile(dset.getTile({blockCol, blockRow}));
        cv::Mat image(dset.tileSize_.height, dset.tileSize_.width
                      , CV_8UC1, rawImage);
        tile.copyTo(image);
    } catch (const std::exception &e) {
        CPLError(CE_Failure, CPLE_FileIO, "%s\n", e.what());
        return CE_Failure;
    }
    return CE_None;
}

} // namespace gdal_drivers

/* GDALRegister_BorderedArea */

void GDALRegister_BorderedArea()
{
    if (!GDALGetDriverByName("BorderedArea")) {
        std::unique_ptr<GDALDriver> driver(new GDALDriver());

        driver->SetDescription("BorderedArea");
        driver->SetMetadataItem
            (GDAL_DMD_LONGNAME
             , "Support for mask defined as area with detailed border.");
        driver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        driver->pfnOpen = gdal_drivers::BorderedAreaDataset::Open;

        GetGDALDriverManager()->RegisterDriver(driver.release());
    }
}
