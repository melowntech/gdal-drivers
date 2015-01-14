/*
 * gttdataset.cpp
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

#include "./gttdataset.hpp"


namespace gdal_drivers {

namespace ut = utility;

namespace {

struct Config {
    double tileUnits;
    uint tilePixels;
    GttDataset::TileType tileType;
    math::Point2 alignment;
    geo::SrsDefinition srs;

    Config() : tileUnits(), tilePixels(), tileType() {}
};

Config parseGttConfig_v1(const Json::Value &config)
{
    Config cfg;
    // tile unit, tile pixels, tile type
    cfg.tileUnits = config["tileUnits"].asDouble();
    cfg.tilePixels = config["tilePixels"].asUInt();

    cfg.tileType = boost::lexical_cast<GttDataset::TileType>
        (config["tileType"].asString());

    // alignment
    cfg.alignment = math::Point2(config["alignment"][0].asDouble(),
                                  config["alignment"][1].asDouble());

    // srs
    cfg.srs = geo::SrsDefinition(config["srs"].asString()
                                 , geo::SrsDefinition::Type::proj4);

    return cfg;
}

Config parseGttConfig(const Json::Value &config)
{
    try {
        // version
        auto version(config["version"].asInt());
        switch (version) {
        case 1: return parseGttConfig_v1(config);
        }
        utility::raise<std::runtime_error>("Unsupported GTT config version "
                                           , version);
    } catch ( const Json::Error & e  ) {
        utility::raise<std::runtime_error>("Invalid format: <%s>.", e.what());
    }

    throw; // never reached
}

void buildIndex(const fs::path &path, const fs::path &tileIndexPath
                , const Json::Value &config)
{
    auto cfg(parseGttConfig(config));
    const auto alignment(cfg.alignment);
    const auto tileUnits(cfg.tileUnits);
    const auto tilePixels(cfg.tilePixels);

    struct File {
        math::Point2i index;
        fs::path name;
        math::Extents2 extents;

        typedef std::vector<File> list;

        File(const math::Point2i &index, const fs::path &name
             , const math::Extents2 &extents)
            : index(index), name(name), extents(extents)
        {}
    };

    File::list files;

    math::Extents2i indexExtents(math::InvalidExtents{});

    // go to dir
    struct CurrentPath {
        CurrentPath(const fs::path &path)
            : path(path), old(fs::current_path())
        {
            fs::current_path(path);
        }
        ~CurrentPath() { fs::current_path(old); }

        fs::path path;
        fs::path old;
    };

    {
        CurrentPath guard(path);

        // traverse directory structure and process all files
        for (fs::recursive_directory_iterator ifrom("."), efrom;
             ifrom != efrom; ++ifrom)
        {
            auto file(ifrom->path());
            if (file.extension().string() != ".tif") {
                // only tif's are allowed
                continue;
            }

            // open dataset
            auto ds(geo::GeoDataset::createFromFS(file));

            if (!areSame(cfg.srs, ds.srs())) {
                utility::raise<std::runtime_error>
                    ("Tile from %s is in differente SRS (%s) "
                     "than GTT SRS (%s)."
                     , file, ds.srsProj4(), cfg.srs.srs);
            }

            // get extents
            auto de(ds.extents());
            auto size(ds.size());
            if (cfg.tileType == GttDataset::TileType::Grid) {
                // grid -> transform extents to pixel-based tile
                auto res(ds.resolution());
                res /= 2.0;
                de.ll += res;
                de.ur -= res;

                size.width -= 1;
                size.height -= 1;
            }

            if ((size.width != static_cast<int>(tilePixels))
                || (size.height != static_cast<int>(tilePixels)))
                {
                    utility::raise<std::runtime_error>
                        ("Tile size (%s) of file %s mismatches with "
                         "config.tilePixels (%d)."
                         , size, file, tilePixels);
                }

            math::Point2 dindex((de.ll(0) - alignment(0)) / tileUnits
                                , de.ll(1) - alignment(1) / tileUnits);

            math::Point2i index(int(std::round(dindex(0)))
                                , int(std::round(dindex(1))));

            // TODO: check index to be really close to dindex

            files.emplace_back(index, file, de);

            // remember tile in index extents
            update(indexExtents, index);
        }
    }

    if (files.empty()) {
        utility::raise<std::runtime_error>
            ("No dataset found in this GTT.");
    }

    try {
        std::ofstream f;
        f.exceptions(std::ios::badbit | std::ios::failbit);
        f.open(tileIndexPath.string());

        // dump
        for (auto &file : files) {
            // fix indixes
            file.index(0) -= indexExtents.ll(0);
            file.index(1) = indexExtents.ur(1) - file.index(1);

            f << file.index(0) << ',' << file.index(1)
              << ',' << file.name.string()
              << ',' << file.extents.ll(0)
              << ',' << file.extents.ll(1)
              << ',' << file.extents.ur(0)
              << ',' << file.extents.ur(1)
              << '\n';
        }
        f.close();
    } catch (const std::exception) {
        utility::raise<std::runtime_error>
            ("Error writing to %s.", tileIndexPath);
    }
}

} // namespace

/* class GttDataset */

GDALDataset * GttDataset::Open( GDALOpenInfo * openInfo ) {

    GDALDataset * dset( 0x0 );
    fs::path path( openInfo->pszFilename );
    Json::Value config;

    // check whether filename is a directory
    if ( ! openInfo->bIsDirectory ) {
        // skip driver
        return 0x0;
    }

    // look for configuration file
    std::ifstream f;
    f.exceptions( std::ios::badbit | std::ios::failbit );

    try {

        f.open( ( path / "gtt_config.json" ).string(), std::ios_base::in );
        f.exceptions( std::ios::badbit );

    } catch ( std::ifstream::failure ) {

        // skip driver
        return 0x0;
    }

    // ok, we take ownership, no silent skipping from now on

    // no updates
    if( openInfo->eAccess == GA_Update ) {

        CPLError( CE_Failure, CPLE_NotSupported,
                  "The JDEM driver does not support update access to existing"
                  " datasets.\n" );
        return 0x0;
    }

    // read configuration file
    try {

        Json::Reader reader;

        if ( ! reader.parse( f, config ) ) {

            CPLError( CE_Failure, CPLE_IllegalArg,
                      "Failed to parse GTT configuration file (%s).\n",
                      reader.getFormatedErrorMessages().c_str() );
            return 0x0;
        }

        f.close();

    } catch ( std::exception & ) {

        CPLError( CE_Failure, CPLE_FileIO, "Unable to read GTT config_file.\n" );
        return 0x0;
    }

    // check for tile index
    fs::path tileIndexPath = path / "gtt_index.csv";

    if ( ! exists( tileIndexPath ) ) {
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

        ut::separated_values::parse( tileIndexPath, ",",
            [&]( const std::vector<std::string> & values ) {
                tileIndex.emplace_back( values );
            } );

    }  catch ( const std::exception & e) {

        CPLError( CE_Failure, CPLE_IllegalArg
                  , "Could not parse GTT tile index: <%s>.\n"
                  , e.what());
        return nullptr;
    }


    // initialize dataset
    try {

        dset = new GttDataset( path, config, tileIndex );

    } catch ( std::runtime_error & e ) {

        CPLError( CE_Failure, CPLE_IllegalArg,
            "Dataset initialization failure (%s).\n", e.what() );
        return nullptr;
    }

    // all done
    return dset;
}


GttDataset::GttDataset(
    const fs::path & path,
    const Json::Value & config,
    const std::vector<std::vector<std::string > > & tileIndex )
    : path_( path ), srs_( "" ), srsWkt_( "" )
{
    {
        auto cfg(parseGttConfig(config));
        tileUnits_ = cfg.tileUnits;
        tilePixels_ = cfg.tilePixels;
        tileType_ = cfg.tileType;
        alignment_ = cfg.alignment;
        srs_ = cfg.srs;
    }

    srsWkt_ = srs_.as(geo::SrsDefinition::Type::wkt);

    // process tile index
    cols_ = 0; rows_ = 0;

    for ( std::vector<std::string> tileinfo : tileIndex ) {

        TileId tileId;
        TileDesc tileDesc;

        try {

            tileId = TileId(
                boost::lexical_cast<uint>(tileinfo[0]),
                boost::lexical_cast<uint>(tileinfo[1]) );

            tileDesc = TileDesc(
                tileinfo[2],
                boost::lexical_cast<double>(tileinfo[3]),
                boost::lexical_cast<double>(tileinfo[4]),
                boost::lexical_cast<double>(tileinfo[5]),
                boost::lexical_cast<double>(tileinfo[6]) );

        } catch ( boost::bad_lexical_cast & e ) {
            utility::raise<std::runtime_error>
                ("Invalid format: <%s>.", e.what());
        }

        tileIndex_[ tileId ] = tileDesc;

        if ( empty( extents_ ) ) {
            extents_ = tileDesc.extents;
        } else {
            extents_  = unite( extents_, tileDesc.extents );
        }

        cols_ = std::max( cols_, tileId.x + 1 );
        rows_ = std::max( rows_, tileId.y + 1 );
    }

    // raster size
    if ( tileType_ == TileType::Grid ) {

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

        for ( uint i = 1; i <= channels.size(); i++ ) {

            SetBand( i, new GttRasterBand( this, i, channels[i-1] ) );
        }

    } catch ( const Json::Error & e  ) {
        utility::raise<std::runtime_error>("Invalid format: <%s>.", e.what());
    }

    // done
}

CPLErr GttDataset::GetGeoTransform( double * padfTransform ) {

    padfTransform[0] = extents_.ll[0];
    padfTransform[1] = tileUnits_ / tilePixels_;
    padfTransform[2] = 0.0;

    padfTransform[3] = extents_.ur[1];
    padfTransform[4] = 0.0;
    padfTransform[5] = - tileUnits_ / tilePixels_;

    return CE_None;
}

const char * GttDataset::GetProjectionRef() {

    return srsWkt_.srs.c_str();
}

/* GttRasterBand */

GttRasterBand::GttRasterBand( GttDataset * dset, int numBand,
    const Json::Value & channel ) {

    // dset, nband
    poDS = dset; nBand = numBand;

    // block size
    nBlockXSize = nBlockYSize = dset->tilePixels_;

    // data type
    eDataType = dataType( channel["dataType"] );

    // color interp
    colorInterp_ = colorInterp( channel["colorInterp"] );

    // no data value
    noDataValue_ = noDataValue( channel["noDataValue"] );


    /*
    LOG( debug ) << poBlocks;
    LOG( debug ) << nRasterXSize << nRasterYSize;
    LOG( debug ) << nBlocksPerRow << " " << nBlocksPerColumn;*/
}

GDALDataType GttRasterBand::dataType( const Json::Value & value ) {

    if ( value.isNull() ) throw std::runtime_error(
        "channel without data type" );

    std::string val = value.asString();

    if ( val == "Byte" ) return GDT_Byte;
    if ( val == "UInt16" ) return GDT_UInt16;
    if ( val == "Int16" ) return GDT_Int16;
    if ( val == "UInt32" ) return GDT_UInt32;
    if ( val == "Int32" ) return GDT_Int32;
    if ( val == "Float32" ) return GDT_Float32;
    if ( val == "Float64" ) return GDT_Float64;

    throw std::runtime_error( "unknown channel data type" );
}


boost::optional<GDALColorInterp> GttRasterBand::colorInterp(
    const Json::Value & value ) {

    if ( value.isNull() ) return boost::none;
    std::string val = value.asString();

    if ( val == "GrayIndex" ) return GCI_GrayIndex;
    if ( val == "PaletteIndex" ) return GCI_PaletteIndex;
    if ( val == "RedBand" ) return GCI_RedBand;
    if ( val == "GreenBand" ) return GCI_GreenBand;
    if ( val == "BlueBand" ) return GCI_BlueBand;
    if ( val == "AlphaBand" ) return GCI_BlueBand;

    throw std::runtime_error( "unknown color interpretation for channel" );
}


boost::optional<double> GttRasterBand::noDataValue( const Json::Value & value ) {

    if ( value.isNull() ) return boost::none;

    return value.asDouble();
}

double GttRasterBand::GetNoDataValue( int * success ) {

    if ( ! noDataValue_ ) {

        if ( success ) *success = 0;
        return std::numeric_limits<double>::max();
    }

    if ( success ) *success = 1;
    return *noDataValue_;
}

GDALColorInterp GttRasterBand::GetColorInterpretation() {

    if ( ! colorInterp_ ) return GCI_Undefined;
    return *colorInterp_;
}


CPLErr GttRasterBand::IReadBlock( int blockCol, int blockRow,
                                  void * image ) {

    uint offsetX(0), offsetY(0);
    GttDataset * dset = static_cast<GttDataset *>( poDS );

    // special case: grid registration, last row/column
    if ( dset->tileType_ == GttDataset::TileType::Grid ) {

        if ( blockCol == (int) dset->cols_ ) {
            blockCol--; offsetX = dset->tilePixels_ - 1;
        }

        if ( blockRow == (int) dset->rows_ ) {
            blockRow--; offsetY = dset->tilePixels_ - 1;
        }
    }

    // does tile exist?
    auto it( dset->tileIndex_.find(
        GttDataset::TileId( blockCol, blockRow ) ) );

    // if not, return no data values (or zeros)
    if ( it == dset->tileIndex_.end() )
        return readEmptyBlock( image );

    // reference to tile
    GttDataset::TileDesc & tileDesc( it->second );

    // open dataset
    GDALDataset * bldset = (GDALDataset *) GDALOpen(
        ( dset->path_ / tileDesc.path ).string().c_str(), GA_ReadOnly );

    if ( ! bldset )
        return CE_Failure;

    std::shared_ptr<GDALDataset>  ldset( bldset );

    // sanity
    ut::expect(
        ldset->GetRasterCount() == dset->GetRasterCount() &&
        ldset->GetRasterBand( nBand )->GetColorInterpretation()
            == GetColorInterpretation(), "Unexpected inconsistency" );

    ut::expect(
        ldset->GetRasterXSize() == (int) dset->tilePixels_
            + ( dset->tileType_ == GttDataset::TileType::Grid ? 1 : 0  ) &&
        ldset->GetRasterYSize() == (int) dset->tilePixels_
            + ( dset->tileType_ == GttDataset::TileType::Grid ? 1 : 0  ),
        "Unexpected inconsistency" );

    // data transfer
    return readBlock( image, ldset.get(), offsetX, offsetY );
}


CPLErr GttRasterBand::readEmptyBlock( void * image )
{
    GttDataset * dset = static_cast<GttDataset *>( poDS );

    double nodata = noDataValue_ ? *noDataValue_ : 0.0;

    if ( eDataType == GDT_Byte ) {

        unsigned char * data = (unsigned char *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<unsigned char>( nodata ) );
        return CE_None;
    }

    if ( eDataType == GDT_UInt16 ) {

        unsigned short * data = (unsigned short *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<unsigned short>( nodata ) );
        return CE_None;
    }

    if ( eDataType == GDT_Int16 ) {

        short * data = (short *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<short>( nodata ) );
        return CE_None;
    }

    if ( eDataType == GDT_UInt32 ) {

        uint * data = (uint *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<uint>( nodata ) );
        return CE_None;
    }

    if ( eDataType == GDT_Int32 ) {

        int * data = (int *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<int>( nodata ) );
        return CE_None;
    }

    if ( eDataType == GDT_Float32 ) {

        float * data = (float *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<float>( nodata ) );
        return CE_None;
    }

    if ( eDataType == GDT_Float64 ) {

        double * data = (double *) image;
        std::fill( data, data + dset->tilePixels_ * dset->tilePixels_,
                   static_cast<double>( nodata ) );
        return CE_None;
    }

    return CE_Failure;
}


CPLErr GttRasterBand::readBlock( void * image, GDALDataset * ldset,
    uint offsetX, uint offsetY ) {

    GttDataset * dset = static_cast<GttDataset *>( poDS );

    return( ldset->GetRasterBand(nBand)->RasterIO(
                    GF_Read,
                    offsetX, offsetY,
                    dset->tilePixels_ - offsetX,
                    dset->tilePixels_ - offsetY,
                    (void *) image,
                    dset->tilePixels_,
                    dset->tilePixels_,
                    eDataType,
                    0, 0 ) );
}

} // namespace gdal_drivers

/* GDALRegister_Gtt */

void GDALRegister_Gtt()
{
    GDALDriver  *poDriver;

    if ( GDALGetDriverByName( "GTT" ) == NULL ) {

        poDriver = new GDALDriver();

        poDriver->SetDescription( "GTT" );
        poDriver->SetMetadataItem( GDAL_DMD_LONGNAME,
                                   "GeoTIFF tiles" );
        poDriver->SetMetadataItem( GDAL_DMD_EXTENSION, "" );

        poDriver->pfnOpen = gdal_drivers::GttDataset::Open;

        GetGDALDriverManager()->RegisterDriver( poDriver );
    }
}
