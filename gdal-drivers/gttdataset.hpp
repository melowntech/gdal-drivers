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
 * @file gttdataset.hpp
 * @author Ondrej Prochazka <ondrej.prochazka@citationtech.net>
 *
 * GTT (GeoTIFF tiles) GDAL driver implementation
 *
 * GeoTIFF tiles is a virtual dataset composed on the fly from a set of
 * tiles forming a non-overlapping, regularly spaced grid. A good example
 * is a set of degree tiles. Both grid registered and pixel registered tiles
 * are supported. The tiles have to be georeferenced, the directory in which
 * they are present needs to be writable and, most importantly, a configuration
 * file named gtt_config.json needs to be present.
 *
 * Pulled in gdal_drivers by Vaclav Blazek <vaclav.blazek@citationtech.net>
 */

#ifndef gdal_drivers_gttdataset_hpp_included_
#define gdal_drivers_gttdataset_hpp_included_

#include <gdal_priv.h>

#include <map>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include "jsoncpp/json.hpp"
#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"
#include "utility/enum-io.hpp"

namespace gdal_drivers {

namespace fs = boost::filesystem;

class GttRasterBand;

/**
 * @brief GttDataset
 */

class GttDataset : public GDALDataset {

friend class GttRasterBand;

public:
    static GDALDataset * Open( GDALOpenInfo * openInfo );

    virtual ~GttDataset() {};

    virtual CPLErr GetGeoTransform( double * padfTransform );
    virtual const char *GetProjectionRef();

    enum class TileType {
        Grid, Pixel
    };

private:

    GttDataset(
        const fs::path & path,
        const Json::Value & config,
        const std::vector<std::vector<std::string > > & tileIndex );

    struct TileId {
        uint x,y;

        TileId() : x(), y() {}

        TileId( uint x, uint y ) : x(x), y(y) {}

        bool operator < ( const TileId & op ) const {
            return x < op.x  || ( x == op.x && y < op.y );
        }
    };

    struct TileDesc {
        fs::path path;
        math::Extents2 extents;

        TileDesc() : path(), extents() {}

        TileDesc( const fs::path & path,
                  double llx, double lly, double urx, double ury )
            : path( path ), extents( llx, lly, urx, ury ) {}
    };

    fs::path path_;
    uint tilePixels_;
    double tileUnits_;
    TileType tileType_;
    math::Point2 alignment_;
    std::map<TileId,TileDesc> tileIndex_;
    math::Extents2 extents_;
    uint cols_, rows_;
    geo::SrsDefinition srs_, srsWkt_;
};

/**
 * @brief GttRasterBand
 */

class GttRasterBand : public GDALRasterBand
{

public:
    GttRasterBand( GttDataset * dset, int numBand,
                   const Json::Value & channel );

    virtual double GetNoDataValue( int * success = 0x0 );
    virtual GDALColorInterp GetColorInterpretation();

    virtual CPLErr IReadBlock( int blockCol, int blockRow, void * image );

    virtual ~GttRasterBand() {};

private:
    boost::optional<double> noDataValue_;
    boost::optional<double> defaultValue_;
    boost::optional<GDALColorInterp> colorInterp_;

    static GDALDataType dataType( const Json::Value & value );
    static boost::optional<GDALColorInterp> colorInterp( const Json::Value & value );
    CPLErr readEmptyBlock( void * image );
    CPLErr readBlock( void * image, GDALDataset * ldset,
        uint offsetX, uint offsetY );

};

// Add iostream input/output operators for tile type
UTILITY_GENERATE_ENUM_IO(GttDataset::TileType,
                         ((Grid)("grid"))
                         ((Pixel)("pixel"))
                         )

} // namespace gdal_drivers

CPL_C_START
void GDALRegister_Gtt(void);
CPL_C_END

#endif // gdal_drivers_gttdataset_hpp_included_
