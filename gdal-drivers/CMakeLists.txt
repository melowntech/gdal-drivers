define_module(LIBRARY gdal-drivers DEPENDS geo jsoncpp CURL)

set(gdal-drivers_SOURCES
  register.hpp register.cpp

  gttdataset.hpp gttdataset.cpp
  mapy-cz.hpp mapy-cz.cpp
  webmerc.hpp webmerc.cpp
  borderedarea.hpp borderedarea.cpp

  detail/fetcher.hpp detail/fetcher.cpp
  detail/localcache.hpp detail/localcache.cpp
)

add_library(gdal-drivers STATIC ${gdal-drivers_SOURCES})
buildsys_library(gdal-drivers)

target_link_libraries(gdal-drivers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(gdal-drivers ${MODULE_DEFINITIONS})
