#include "./mapy-cz.hpp"
#include "./gttdataset.hpp"

namespace gdal_drivers {

void registerAll()
{
    // put new drivers here
    GDALRegister_MapyCz();
    GDALRegister_Gtt();
}

} // namespace gdal_drivers
