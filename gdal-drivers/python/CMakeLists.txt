define_module(LIBRARY pygdal-drivers=${MODULE_gdal-drivers_VERSION}
  DEPENDS
  gdal-drivers=${MODULE_gdal-drivers_VERSION}
  math>=1.10 # optional sizes
  imgproc>=1.29 # needs numpy support
  geo
  pysupport>=1.6
  NumPy OpenCV
  Boost_THREAD Boost_SYSTEM)

set(pygdal-drivers_SOURCES
  gdaldriversmodule.hpp
  gdaldriversmodule.cpp
  )

add_library(pygdal-drivers STATIC ${pygdal-drivers_SOURCES})
buildsys_library(pygdal-drivers)
target_compile_definitions(pygdal-drivers PRIVATE ${MODULE_DEFINITIONS})
target_link_libraries(pygdal-drivers ${MODULE_LIBRARIES})
