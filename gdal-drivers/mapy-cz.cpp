/*
 * mapy-cz.cpp
 */

#include <limits>
#include <algorithm>
#include <vector>
#include <boost/lexical_cast.hpp>

#include "utility/parse.hpp"
#include "utility/expect.hpp"
#include "utility/raise.hpp"
#include "dbglog/dbglog.hpp"
#include "geo/geodataset.hpp"

#include "./mapy-cz.hpp"

namespace gdal_drivers {


namespace ut = utility;

/* class MapyczDataset */

GDALDataset* MapyczDataset::Open(GDALOpenInfo* openInfo)
{
    (void) openInfo;

    GDALDataset *dset(nullptr);
    // parse filename
#if 0
    fs::path path(openInfo->pszFilename);

    // check whether filename is a directory
    if (!openInfo->bIsDirectory) {
        // skip driver
        return 0x0;
    }

    // look for configuration file
    std::ifstream f;
    f.exceptions(std::ios::badbit | std::ios::failbit);

    try {

        f.open((path / "gtt_config.json").string(), std::ios_base::in);
        f.exceptions(std::ios::badbit);

    } catch (std::ifstream::failure) {

        // skip driver
        return 0x0;
    }

    // ok, we take ownership, no silent skipping from now on

    // no updates
    if(openInfo->eAccess == GA_Update) {

        CPLError(CE_Failure, CPLE_NotSupported,
                  "The JDEM driver does not support update access to existing"
                  " datasets.\n");
        return 0x0;
    }

    // read configuration file
    try {

        Json::Reader reader;

        if (!reader.parse(f, config)) {

            CPLError(CE_Failure, CPLE_IllegalArg,
                      "Failed to parse GTT configuration file (%s).\n",
                      reader.getFormatedErrorMessages().c_str());
            return 0x0;
        }

        f.close();

    } catch (std::exception &) {

        CPLError(CE_Failure, CPLE_FileIO, "Unable to read GTT config_file.\n");
        return 0x0;
    }

    // check for tile index
    fs::path tileIndexPath = path / "gtt_index.csv";

    if (!exists(tileIndexPath)) {
        // TODO: catch error
        try {
            buildIndex(path, tileIndexPath, config);
        } catch (const std::exception &e) {
            CPLError(CE_Failure, CPLE_FileIO
                     , "Unable to build GTT tile index: <%s>.\n"
                     , e.what());
            return nullptr;
        }
    }

    // read tile index
    std::vector<std::vector<std::string> > tileIndex;

    try {

        ut::separated_values::parse(tileIndexPath, ",",
            [&](const std::vector<std::string> & values) {
                tileIndex.emplace_back(values);
            });

    }  catch (const std::exception & e) {

        CPLError(CE_Failure, CPLE_IllegalArg
                  , "Could not parse GTT tile index: <%s>.\n"
                  , e.what());
        return nullptr;
    }


    // initialize dataset
    try {

        dset = new MapyczDataset(path, config, tileIndex);

    } catch (std::runtime_error & e) {

        CPLError(CE_Failure, CPLE_IllegalArg,
            "Dataset initialization failure (%s).\n", e.what());
        return nullptr;
    }

    // all done
    return dset;
#endif
    return dset;
}


MapyczDataset::MapyczDataset()
{
#if 0
    srsWkt_ = srs_.as(geo::SrsDefinition::Type::wkt);

    // process tile index
    cols_ = 0; rows_ = 0;

    for (std::vector<std::string> tileinfo : tileIndex) {

        TileId tileId;
        TileDesc tileDesc;

        try {

            tileId = TileId(
                boost::lexical_cast<uint>(tileinfo[0]),
                boost::lexical_cast<uint>(tileinfo[1]));

            tileDesc = TileDesc(
                tileinfo[2],
                boost::lexical_cast<double>(tileinfo[3]),
                boost::lexical_cast<double>(tileinfo[4]),
                boost::lexical_cast<double>(tileinfo[5]),
                boost::lexical_cast<double>(tileinfo[6]));

        } catch (boost::bad_lexical_cast & e) {
            utility::raise<std::runtime_error>
                ("Invalid format: <%s>.", e.what());
        }

        tileIndex_[ tileId ] = tileDesc;

        if (empty(extents_)) {
            extents_ = tileDesc.extents;
        } else {
            extents_  = unite(extents_, tileDesc.extents);
        }

        cols_ = std::max(cols_, tileId.x + 1);
        rows_ = std::max(rows_, tileId.y + 1);
    }

    // raster size
    if (tileType_ == TileType::Grid) {

        // grid registration
        nRasterXSize = cols_ * tilePixels_ + 1;
        nRasterYSize = rows_ * tilePixels_ + 1;

        extents_.ur[0] += 0.5 * tileUnits_ / tilePixels_;
        extents_.ur[1] += 0.5 * tileUnits_ / tilePixels_;

        extents_.ll[0] = extents_.ur[0] - cols_ * tileUnits_
            - tileUnits_ / tilePixels_;
        extents_.ll[1] = extents_.ur[1] - rows_ * tileUnits_
            - tileUnits_ / tilePixels_;

    } else {

        // pixel registration
        nRasterXSize = cols_ * tilePixels_;
        nRasterYSize = rows_ * tilePixels_;

        extents_.ll[0] = extents_.ur[0] - cols_ * tileUnits_;
        extents_.ll[1] = extents_.ur[1] - rows_ * tileUnits_;
    }

    // set up channels
    try {

        // channels
        Json::Value channels = config["channels"];

        for (uint i = 1; i <= channels.size(); i++) {

            SetBand(i, new MapyczRasterBand(this, i, channels[i-1]));
        }

    } catch (const Json::Error & e ) {
        utility::raise<std::runtime_error>("Invalid format: <%s>.", e.what());
    }

    // done
#endif
}

CPLErr MapyczDataset::GetGeoTransform(double * padfTransform) {

    padfTransform[0] = extents_.ll[0];
    padfTransform[1] = tileUnits_ / tilePixels_;
    padfTransform[2] = 0.0;

    padfTransform[3] = extents_.ur[1];
    padfTransform[4] = 0.0;
    padfTransform[5] = - tileUnits_ / tilePixels_;

    return CE_None;
}

const char * MapyczDataset::GetProjectionRef() {

    return srsWkt_.srs.c_str();
}

/* MapyczRasterBand */

MapyczRasterBand::MapyczRasterBand(MapyczDataset * dset, int numBand
                                    , int channel)
{
    (void) dset; (void) numBand; (void) channel;
#if 0
    // dset, nband
    poDS = dset; nBand = numBand;

    // block size
    nBlockXSize = nBlockYSize = dset->tilePixels_;

    // data type
    eDataType = dataType(channel["dataType"]);

    // color interp
    colorInterp_ = colorInterp(channel["colorInterp"]);

    // no data value
    noDataValue_ = noDataValue(channel["noDataValue"]);
#endif
}

GDALColorInterp MapyczRasterBand::GetColorInterpretation() {

    if (!colorInterp_) return GCI_Undefined;
    return *colorInterp_;
}


CPLErr MapyczRasterBand::IReadBlock(int blockCol, int blockRow,
                                  void * image) {

    uint offsetX(0), offsetY(0);
    MapyczDataset * dset = static_cast<MapyczDataset *>(poDS);
#if 0
    // special case: grid registration, last row/column
    if (dset->tileType_ == MapyczDataset::TileType::Grid) {

        if (blockCol == (int) dset->cols_) {
            blockCol--; offsetX = dset->tilePixels_ - 1;
        }

        if (blockRow == (int) dset->rows_) {
            blockRow--; offsetY = dset->tilePixels_ - 1;
        }
    }
#endif

    // does tile exist?
    auto it(dset->tileIndex_.find(
        MapyczDataset::TileId(blockCol, blockRow)));

    // if not, return no data values (or zeros)
    if (it == dset->tileIndex_.end())
        return readEmptyBlock(image);

    // reference to tile
    MapyczDataset::TileDesc & tileDesc(it->second);

    // open dataset
    GDALDataset * bldset = (GDALDataset *) GDALOpen(
        (dset->path_ / tileDesc.path).string().c_str(), GA_ReadOnly);

    if (!bldset)
        return CE_Failure;

    std::shared_ptr<GDALDataset>  ldset(bldset);

    // sanity
    ut::expect(
        ldset->GetRasterCount() == dset->GetRasterCount() &&
        ldset->GetRasterBand(nBand)->GetColorInterpretation()
            == GetColorInterpretation(), "Unexpected inconsistency");

    // ut::expect(
    //     ldset->GetRasterXSize() == (int) dset->tilePixels_
    //         + (dset->tileType_ == MapyczDataset::TileType::Grid ? 1 : 0 ) &&
    //     ldset->GetRasterYSize() == (int) dset->tilePixels_
    //         + (dset->tileType_ == MapyczDataset::TileType::Grid ? 1 : 0 ),
    //     "Unexpected inconsistency");

    // data transfer
    return readBlock(image, ldset.get(), offsetX, offsetY);
}


CPLErr MapyczRasterBand::readEmptyBlock(void * image)
{
    MapyczDataset * dset = static_cast<MapyczDataset *>(poDS);

    double nodata = noDataValue_ ? *noDataValue_ : 0.0;

    if (eDataType == GDT_Byte) {

        unsigned char * data = (unsigned char *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<unsigned char>(nodata));
        return CE_None;
    }

    if (eDataType == GDT_UInt16) {

        unsigned short * data = (unsigned short *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<unsigned short>(nodata));
        return CE_None;
    }

    if (eDataType == GDT_Int16) {

        short * data = (short *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<short>(nodata));
        return CE_None;
    }

    if (eDataType == GDT_UInt32) {

        uint * data = (uint *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<uint>(nodata));
        return CE_None;
    }

    if (eDataType == GDT_Int32) {

        int * data = (int *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<int>(nodata));
        return CE_None;
    }

    if (eDataType == GDT_Float32) {

        float * data = (float *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<float>(nodata));
        return CE_None;
    }

    if (eDataType == GDT_Float64) {

        double * data = (double *) image;
        std::fill(data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<double>(nodata));
        return CE_None;
    }

    return CE_Failure;
}


CPLErr MapyczRasterBand::readBlock(void * image, GDALDataset * ldset,
    uint offsetX, uint offsetY) {

    MapyczDataset * dset = static_cast<MapyczDataset *>(poDS);

    return(ldset->GetRasterBand(nBand)->RasterIO(
                    GF_Read,
                    offsetX, offsetY,
                    dset->tilePixels_ - offsetX,
                    dset->tilePixels_ - offsetY,
                    (void *) image,
                    dset->tilePixels_,
                    dset->tilePixels_,
                    eDataType,
                    0, 0));
}

} // namespace gdal_drivers

/* GDALRegister_Mapycz */

void GDALRegister_MapyCz()
{
    if (!GDALGetDriverByName("mapy.cz")) {
        GDALDriver *poDriver = new GDALDriver();

        poDriver->SetDescription("mapy.cz");
        poDriver->SetMetadataItem(GDAL_DMD_LONGNAME,
                                   "GeoTIFF tiles");
        poDriver->SetMetadataItem(GDAL_DMD_EXTENSION, "");

        poDriver->pfnOpen = gdal_drivers::MapyczDataset::Open;

        GetGDALDriverManager()->RegisterDriver(poDriver);
    }
}
