# bump version here
set(gdal-drivers_VERSION 1.15)

set(gdal-drivers_DEPENDS)
set(gdal-drivers_DEFINITIONS)

set(gdal-drivers_SOURCES
  register.hpp register.cpp

  mask.hpp mask.cpp
  solid.hpp solid.cpp
  blender.hpp blender.cpp

  detail/mbtiles.hpp
  detail/extents.hpp
  detail/geotransform.hpp
  detail/srsholder.hpp
  )

if(PROTOBUF_FOUND)
  message(STATUS "gdal-drivers: compiling in protobuf support")
  list(APPEND gdal-drivers_SOURCES
    mvt.hpp mvt.cpp)

  list(APPEND gdal-drivers_DEPENDS PROTOBUF)
  list(APPEND gdal-drivers_DEFINITIONS GDAL_DRIVERS_HAS_PROTOBUF)
  protobuf_generate_cpp(gdal-drivers_PROTO_SOURCES gdal-drivers_PROTO_HDRS
    proto/vector_tile.proto)

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
else()
  message(STATUS "gdal-drivers: compiling without protobuf support")
endif()

define_module(LIBRARY gdal-drivers=${gdal-drivers_VERSION}
  DEPENDS
  ${gdal-drivers_DEPENDS}
  geo>=1.31 jsoncpp>=2.1
  Boost_PROGRAM_OPTIONS
  DEFINITIONS ${gdal-drivers_DEFINITIONS}
  )

# protobuf generator generates that triggers warning in recent gcc -> no
# warnings
if(${CMAKE_CXX_COMPILER_ID} MATCHES GNU)
  foreach(source ${gdal-drivers_PROTO_SOURCES})
    set_source_files_properties(${source} PROPERTIES
      COMPILE_FLAGS "-w")
  endforeach()
endif()

add_library(gdal-drivers
  STATIC ${gdal-drivers_SOURCES}
  ${gdal-drivers_PROTO_SOURCES})
buildsys_library(gdal-drivers)

target_link_libraries(gdal-drivers ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(gdal-drivers ${MODULE_DEFINITIONS})

foreach (source ${gdal-drivers_PROTO_SOURCES})
  # disable warnings in protobuf-generated code
  set_source_files_properties(${source} PROPERTIES
    COMPILE_FLAGS -w)
endforeach()
