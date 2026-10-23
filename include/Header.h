#pragma once
#include <optional>
#include <string>

namespace odr
{

// OpenDRIVE 1.6.1, 6.6 Georeferencing: offset applied to the inertial coordinates, used instead of changing projection parameters
struct HeaderOffset
{
    HeaderOffset(double x, double y, double z, double hdg);

    double x = 0;
    double y = 0;
    double z = 0;
    double hdg = 0;
};

// OpenDRIVE 1.6.1, 4.3.2 Header
struct Header
{
    Header() = default;

    unsigned int rev_major = 1;
    unsigned int rev_minor = 0;
    std::string  name = "";
    std::string  version = "";
    std::string  date = "";
    std::string  vendor = "";

    std::optional<double> north;
    std::optional<double> south;
    std::optional<double> east;
    std::optional<double> west;

    std::optional<HeaderOffset> offset;
};

} // namespace odr
