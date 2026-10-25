#pragma once
#include "AdditionalData.h"
#include "LaneValidityRecord.h"
#include "Math.hpp"
#include "Mesh.h"

#include <cmath>
#include <optional>
#include <string>
#include <vector>

namespace odr
{

struct RoadObjectRepeat
{
    RoadObjectRepeat(double s0,
                     double length,
                     double distance,
                     double t_start,
                     double t_end,
                     double width_start,
                     double width_end,
                     double height_start,
                     double height_end,
                     double z_offset_start,
                     double z_offset_end,
                     double length_start = NAN,
                     double length_end = NAN,
                     double radius_start = NAN,
                     double radius_end = NAN);

    double s0 = 0;
    double length = 0;
    double distance = 0;
    double t_start = 0;
    double t_end = 0;
    double width_start = 0;
    double width_end = 0;
    double height_start = 0;
    double height_end = 0;
    double z_offset_start = 0;
    double z_offset_end = 0;
    double length_start = NAN; // OpenDRIVE 1.6.1, 11.1, NAN if not set
    double length_end = NAN;
    double radius_start = NAN;
    double radius_end = NAN;

    AdditionalData additional_data;
};

struct RoadObjectCorner
{
    enum class Type
    {
        Local_RelZ, // z relative to road’s reference line
        Local_AbsZ, // absolute z value
        Road
    };

    RoadObjectCorner(int id, Vec3D pt, double height, Type type);

    int    id = 0;
    Vec3D  pt;
    double height = 0;
    Type   type = Type::Road;

    AdditionalData additional_data;
};

struct RoadObjectOutline
{
    RoadObjectOutline(int id, std::string fill_type, std::string lane_type, bool outer, bool closed);

    int         id = 0;
    std::string fill_type = "";
    std::string lane_type = "";
    bool        outer = true;
    bool        closed = true;

    std::vector<RoadObjectCorner> outline;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.3 Object material
struct RoadObjectMaterial
{
    RoadObjectMaterial(std::string surface, std::optional<double> friction, std::optional<double> roughness);

    std::string           surface = "";
    std::optional<double> friction;
    std::optional<double> roughness;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.5 Access rules to parking spaces
struct RoadObjectParkingSpace
{
    RoadObjectParkingSpace(std::string access, std::string restrictions);

    std::string access = "";       // e.g. "all", "car", "handicapped", "bus"
    std::string restrictions = ""; // free text

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.6 Object marking
struct RoadObjectMarking
{
    RoadObjectMarking(std::string side,
                      std::string weight,
                      double      width,
                      std::string color,
                      double      z_offset,
                      double      space_length,
                      double      line_length,
                      double      start_offset,
                      double      stop_offset);

    std::string side = "";   // side of the bounding box, if no outline points are referenced
    std::string weight = ""; // "standard" or "bold"
    double      width = 0;
    std::string color = "";
    double      z_offset = 0;
    double      space_length = 0;
    double      line_length = 0;
    double      start_offset = 0;
    double      stop_offset = 0;

    std::vector<int> corner_references; // ids of outline points

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.7 Object borders
struct RoadObjectBorder
{
    RoadObjectBorder(double width, std::string type, int outline_id, bool use_complete_outline);

    double      width = 0;
    std::string type = ""; // e.g. "concrete", "curb"
    int         outline_id = 0;
    bool        use_complete_outline = true;

    std::vector<int> corner_references; // ids of outline points, if not use_complete_outline

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.8 Object reference
struct RoadObjectReference
{
    RoadObjectReference(std::string road_id, std::string id, double s0, double t0, double z_offset, double valid_length, std::string orientation);

    std::string road_id = "";
    std::string id = ""; // id of the referenced object
    double      s0 = 0;
    double      t0 = 0;
    double      z_offset = 0;
    double      valid_length = 0;
    std::string orientation = "";

    std::vector<LaneValidityRecord> lane_validities;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.9 Tunnels
struct RoadTunnel
{
    RoadTunnel(std::string road_id, std::string id, double s0, double length, std::string name, std::string type);

    std::string           road_id = "";
    std::string           id = "";
    double                s0 = 0;
    double                length = 0;
    std::string           name = "";
    std::string           type = ""; // "standard" or "underpass"
    std::optional<double> lighting;  // [0;1]
    std::optional<double> daylight;  // [0;1]

    std::vector<LaneValidityRecord> lane_validities;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 11.10 Bridges
struct RoadBridge
{
    RoadBridge(std::string road_id, std::string id, double s0, double length, std::string name, std::string type);

    std::string road_id = "";
    std::string id = "";
    double      s0 = 0;
    double      length = 0;
    std::string name = "";
    std::string type = ""; // "concrete", "steel", "brick" or "wood"

    std::vector<LaneValidityRecord> lane_validities;

    AdditionalData additional_data;
};

struct RoadObject
{
    RoadObject(std::string road_id,
               std::string id,
               double      s0,
               double      t0,
               double      z0,
               double      length,
               double      valid_length,
               double      width,
               double      radius,
               double      height,
               double      hdg,
               double      pitch,
               double      roll,
               std::string type,
               std::string name,
               std::string orientation,
               std::string subtype,
               bool        is_dynamic);

    static Mesh3D get_cylinder(const double eps, const double radius, const double height);
    static Mesh3D get_box(const double width, const double length, const double height);

    std::string road_id = "";

    std::string id = "";
    std::string type = "";
    std::string name = "";
    std::string orientation = "";
    std::string subtype = "";

    double s0 = 0;
    double t0 = 0;
    double z0 = 0;
    double length = 0;
    double valid_length = 0;
    double width = 0;
    double radius = 0;
    double height = 0;
    double hdg = 0;
    double pitch = 0;
    double roll = 0;
    bool   is_dynamic = false;

    std::vector<RoadObjectRepeat>   repeats;
    std::vector<RoadObjectOutline>  outlines;
    std::vector<LaneValidityRecord> lane_validities;

    std::vector<RoadObjectMaterial>       materials;
    std::optional<RoadObjectParkingSpace> parking_space;
    std::vector<RoadObjectMarking>        markings;
    std::vector<RoadObjectBorder>         borders;

    AdditionalData additional_data;
};

} // namespace odr