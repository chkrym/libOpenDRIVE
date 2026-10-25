#pragma once
#include "AdditionalData.h"
#include <optional>
#include <string>

namespace odr
{

// OpenDRIVE 1.6.1, 8.5 Road surface / 10.3 Road surface within junctions: reference to OpenCRG data, the CRG data itself is not evaluated
struct SurfaceCRG
{
    SurfaceCRG(std::string file, std::string mode, std::string purpose);

    std::string file = "";
    std::string mode = "";    // "attached", "attached0", "genuine" or "global"
    std::string purpose = ""; // "elevation" (default) or "friction"

    std::optional<double>      s_start;     // road only
    std::optional<double>      s_end;       // road only
    std::optional<std::string> orientation; // road only, "same" or "opposite"

    double s_offset = 0; // road only
    double t_offset = 0; // road only
    double z_offset = 0;
    double z_scale = 1;
    double h_offset = 0; // road only

    AdditionalData additional_data;
};

} // namespace odr
