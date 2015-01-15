/**
 * @file all.hpp
 * @author Vaclav Blazek <vaclav.blazek@citationtech.net>
 *
 * Registers all our GDAL drivers.
 *
 * Users:
 * Call gdal_drivers::registerAll() to register all drivers in GDAL
 * infrastructure.
 *
 * Developers:
 * Add a call to new driver's registration function inside body of
 * gdal_drivers::registerAll() in register.cpp source file.
 */

#ifndef gdal_drivers_register_hpp_included_
#define gdal_drivers_register_hpp_included_

#include <gdal/cpl_port.h>

namespace gdal_drivers {
    void registerAll();
} // namespace gdal_drivers

#endif // gdal_drivers_register_hpp_included_
