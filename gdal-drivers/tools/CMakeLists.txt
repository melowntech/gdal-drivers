define_module(BINARY gdal-drivers-tools DEPENDS gdal-drivers service)

add_executable(gdal-drivers-ba2mask EXCLUDE_FROM_ALL gdal-drivers-ba2mask.cpp)
target_link_libraries(gdal-drivers-ba2mask ${MODULE_LIBRARIES})
buildsys_target_compile_definitions(gdal-drivers-ba2mask PRIVATE ${MODULE_DEFINITIONS})
buildsys_binary(gdal-drivers-ba2mask)
