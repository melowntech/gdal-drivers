#include "./mapy-cz.hpp"
#include "./webmerc.hpp"
#include "./gttdataset.hpp"
#include "./borderedarea.hpp"
#include "./mask.hpp"
#include "./solid.hpp"
#include "./mvt.hpp"

namespace gdal_drivers {

void registerAll()
{
    // put new drivers here
    GDALRegister_MapyCz();
    GDALRegister_WebMercator();
    GDALRegister_Gtt();
    GDALRegister_BorderedArea();
    GDALRegister_MaskDataset();
    GDALRegister_SolidDataset();
    GDALRegister_MvtDataset();
}

} // namespace gdal_drivers
