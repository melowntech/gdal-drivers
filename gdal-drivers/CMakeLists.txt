# bump version here
set(gdal-drivers_VERSION 1.14)

set(gdal-drivers_DEPENDS)

set(gdal-drivers_SOURCES
  register.hpp register.cpp

  gttdataset.hpp gttdataset.cpp
  mapy-cz.hpp mapy-cz.cpp
  webmerc.hpp webmerc.cpp
  borderedarea.hpp borderedarea.cpp
  mask.hpp mask.cpp
  solid.hpp solid.cpp

  mvt.hpp mvt.cpp

  detail/fetcher.hpp detail/fetcher.cpp
  detail/localcache.hpp detail/localcache.cpp
  detail/mbtiles.hpp
  )

if(Sqlite3_FOUND)
  message(STATUS "gdal-drivers: compiling in Sqlite3 support")
  list(APPEND gdal-drivers_SOURCES
    detail/mbtiles.cpp)

  list(APPEND gdal-drivers_DEPENDS Sqlite3 Boost_IOSTREAMS)
else()
  message(STATUS "gdal-drivers: compiling without Sqlite3 support")
  list(APPEND gdal-drivers_SOURCES
    detail/mbtiles.nosqlite.cpp)
endif()

define_module(LIBRARY gdal-drivers=${gdal-drivers_VERSION}
  DEPENDS
  ${gdal-drivers_DEPENDS}
  geo>=1.24 jsoncpp>=2.1
  CURL PROTOBUF
  Boost_PROGRAM_OPTIONS
  )

protobuf_generate_cpp(gdal-drivers_PROTO_SOURCES gdal-drivers_PROTO_HDRS
  proto/vector_tile.proto)

add_library(gdal-drivers
  STATIC ${gdal-drivers_SOURCES}
  ${gdal-drivers_PROTO_SOURCES})
buildsys_library(gdal-drivers)

target_link_libraries(gdal-drivers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(gdal-drivers ${MODULE_DEFINITIONS})

if(MODULE_service_FOUND)
  add_subdirectory(tools EXCLUDE_FROM_ALL)
endif()
