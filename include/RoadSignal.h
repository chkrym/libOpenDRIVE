#pragma once
#include "LaneValidityRecord.h"
#include "Mesh.h"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace odr
{

// OpenDRIVE 1.6.1, 12.2 Signal dependency: this signal controls the output of the signal with the given id
struct RoadSignalDependency
{
    RoadSignalDependency(std::string id, std::string type);

    std::string id = "";
    std::string type = ""; // free text
};

// OpenDRIVE 1.6.1, 12.3 Links between signals and objects (<reference> within <signal>)
struct RoadSignalElementReference
{
    RoadSignalElementReference(std::string element_type, std::string element_id, std::string type);

    std::string element_type = ""; // "object" or "signal"
    std::string element_id = "";
    std::string type = ""; // free text
};

// OpenDRIVE 1.6.1, 12.4 Signal positioning: physical position in inertial coordinates
struct RoadSignalPositionInertial
{
    RoadSignalPositionInertial(double x, double y, double z, double hdg, double pitch, double roll);

    double x = 0;
    double y = 0;
    double z = 0;
    double hdg = 0;
    double pitch = 0;
    double roll = 0;
};

// OpenDRIVE 1.6.1, 12.4 Signal positioning: physical position in road coordinates
struct RoadSignalPositionRoad
{
    RoadSignalPositionRoad(std::string road_id, double s, double t, double z_offset, double h_offset, double pitch, double roll);

    std::string road_id = "";
    double      s = 0;
    double      t = 0;
    double      z_offset = 0;
    double      h_offset = 0;
    double      pitch = 0;
    double      roll = 0;
};

// OpenDRIVE 1.6.1, 12.5 Reuse of signal information (<signalReference> within <signals>)
struct RoadSignalReference
{
    RoadSignalReference(std::string road_id, std::string id, double s0, double t0, std::string orientation);

    std::string road_id = "";
    std::string id = ""; // id of the referenced signal
    double      s0 = 0;
    double      t0 = 0;
    std::string orientation = "";

    std::vector<LaneValidityRecord> lane_validities;
};

// OpenDRIVE 1.6.1, 12.6 Controllers
struct ControllerControl
{
    ControllerControl(std::string signal_id, std::string type);

    std::string signal_id = "";
    std::string type = ""; // free text
};

struct Controller
{
    Controller(std::string id, std::string name, std::optional<std::uint32_t> sequence);

    std::string                  id = "";
    std::string                  name = "";
    std::optional<std::uint32_t> sequence;

    std::vector<ControllerControl> controls;
};

struct RoadSignal
{
    RoadSignal(std::string road_id,
               std::string id,
               std::string name,
               double      s0,
               double      t0,
               bool        is_dynamic,
               double      zOffset,
               double      value,
               double      height,
               double      width,
               double      hOffset,
               double      pitch,
               double      roll,
               std::string orientation,
               std::string country,
               std::string type,
               std::string subtype,
               std::string unit,
               std::string text);

    static Mesh3D get_box(const double width, const double length, const double height);

    std::string road_id = "";
    std::string id = "";
    std::string name = "";
    double      s0 = 0;
    double      t0 = 0;
    bool        is_dynamic = false;
    double      zOffset = 0;
    double      value = 0;
    double      height = 0;
    double      width = 0;
    double      hOffset = 0;
    double      pitch = 0;
    double      roll = 0;
    std::string orientation = "";
    std::string country = "";
    std::string type = "";
    std::string subtype = "";
    std::string unit = "";
    std::string text = "";

    std::vector<LaneValidityRecord> lane_validities;

    std::string                               country_revision = "";
    std::vector<RoadSignalDependency>         dependencies;
    std::vector<RoadSignalElementReference>   references;
    std::optional<RoadSignalPositionInertial> position_inertial;
    std::optional<RoadSignalPositionRoad>     position_road;
};

} // namespace odr
