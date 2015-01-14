define_module(LIBRARY gdal-drivers DEPENDS geo jsoncpp)

set(gdal-drivers_SOURCES
  gttdataset.hpp gttdataset.cpp

  mapy-cz.hpp mapy-cz.cpp
)

add_library(gdal-drivers STATIC ${gdal-drivers_SOURCES})
buildsys_library(gdal-drivers)

target_link_libraries(gdal-drivers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(geo ${MODULE_DEFINITIONS})
