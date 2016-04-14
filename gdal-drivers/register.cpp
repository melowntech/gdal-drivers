#include "./mapy-cz.hpp"
#include "./webmerc.hpp"
#include "./gttdataset.hpp"
#include "./borderedarea.hpp"
#include "./mask.hpp"

namespace gdal_drivers {

void registerAll()
{
    // put new drivers here
    GDALRegister_MapyCz();
    GDALRegister_WebMercator();
    GDALRegister_Gtt();
    GDALRegister_BorderedArea();
    GDALRegister_MaskDataset();
}

} // namespace gdal_drivers
