#include "./mapy-cz.hpp"
#include "./webmerc.hpp"
#include "./gttdataset.hpp"
#include "./borderedarea.hpp"

namespace gdal_drivers {

void registerAll()
{
    // put new drivers here
    GDALRegister_MapyCz();
    GDALRegister_WebMercator();
    GDALRegister_Gtt();
    GDALRegister_BorderedArea();
}

} // namespace gdal_drivers
