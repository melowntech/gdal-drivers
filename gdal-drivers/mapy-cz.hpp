/**
 * @file mapy-cz.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Mapy.cz GDAL driver implementation
 *
 * Adds to GDAL an ability to work with map data from mapy.cz.
 * Dataset uri: mapycz://MAP_TYPE/ZOOM where:
 *    * MAP_TYPE is one of base, ortho
 *    * ZOOM is a level-of-detail specifier in range 0-20
 */


#include <gdal_priv.h>

#include <map>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>

#include "math/geometry_core.hpp"
#include "geo/srsdef.hpp"

namespace fs = boost::filesystem;

namespace gdal_drivers {

class MapyczRasterBand;

/**
 * @brief GttDataset
 */

class MapyczDataset : public GDALDataset {

    friend class MapyczRasterBand;

public:
    static GDALDataset* Open(GDALOpenInfo *openInfo);

    virtual ~MapyczDataset() {};

    virtual CPLErr GetGeoTransform(double *padfTransform);
    virtual const char *GetProjectionRef();

private:
    MapyczDataset();

    struct TileId {
        uint x,y;

        TileId() : x(), y() {}

        TileId(uint x, uint y) : x(x), y(y) {}

        bool operator<(const TileId &op) const {
            return (x < op.x) || ((x == op.x) && (y < op.y));
        }
    };

    struct TileDesc {
        fs::path path;
        math::Extents2 extents;

        TileDesc() : path(), extents() {}

        TileDesc(const fs::path &path
                 , double llx, double lly, double urx, double ury)
            : path(path), extents(llx, lly, urx, ury) {}
    };

    fs::path path_;
    uint tilePixels_;
    double tileUnits_;
    math::Point2 alignment_;
    std::map<TileId,TileDesc> tileIndex_;
    math::Extents2 extents_;
    uint cols_, rows_;
    geo::SrsDefinition srs_, srsWkt_;
};

/**
 * @brief MapyczRasterBand
 */

class MapyczRasterBand : public GDALRasterBand
{

public:
    MapyczRasterBand(MapyczDataset *dset, int numBand
                     , int channel);

    virtual double GetNoDataValue(int *success = nullptr);
    virtual GDALColorInterp GetColorInterpretation();

    virtual CPLErr IReadBlock(int blockCol, int blockRow, void *image);

    virtual ~MapyczRasterBand() {};

private:
    boost::optional<double> noDataValue_;
    boost::optional<GDALColorInterp> colorInterp_;

    CPLErr readEmptyBlock(void *image);
    CPLErr readBlock(void *image, GDALDataset *ldset
                     , uint offsetX, uint offsetY);

};

} // namespace gdal_drivers


// driver registration function
CPL_C_START
void GDALRegister_MapyCz(void);
CPL_C_END
