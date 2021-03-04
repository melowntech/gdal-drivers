/**
 * Copyright (c) 2021 Melown Technologies SE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * *  Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef gdal_drivers_srsholder_hpp_included_
#define gdal_drivers_srsholder_hpp_included_

#include <utility>
#include <string>
#include <gdal_priv.h>

#include "geo/srsdef.hpp"

namespace gdal_drivers {

#if GDAL_VERSION_NUM >= 3000000
class SrsHoldingDataset : public ::GDALDataset {
public:
    SrsHoldingDataset() = default;
    virtual ~SrsHoldingDataset() {}

    template <typename T>
    SrsHoldingDataset(T &&srs) {
        setSrs(std::move(srs));
    }

    virtual const ::OGRSpatialReference* GetSpatialRef() const override {
        return &srs_;
    }

protected:
    void setSrs(const geo::SrsDefinition &srs) { srs_ = srs.reference(); }

    void setSrs(const std::string &srs) {
        srs_ = geo::SrsDefinition(srs, geo::SrsDefinition::Type::wkt)
            .reference();
    }

    ::OGRSpatialReference srs_;
};

#else

class SrsHoldingDataset : public ::GDALDataset {
public:
    SrsHoldingDataset() = default;
    virtual ~SrsHoldingDataset() {}

    template <typename T>
    SrsHoldingDataset(T &&srs) {
        setSrs(std::move(srs));
    }

    virtual const char *GetProjectionRef() override { return srs_.c_str(); }

protected:
    void setSrs(const geo::SrsDefinition &srs) {
        srs_ = srs.as(geo::SrsDefinition::Type::wkt).srs;
    }

    void setSrs(const std::string &srs) { srs_ = srs; }

    std::string srs_;
};

#endif

} // namespace gdal_drivers


#endif // gdal_drivers_srsholder_hpp_included_
