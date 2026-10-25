#pragma once
#include "AdditionalData.h"
#include "Geometries/CubicSpline.h"
#include "RoadMark.h"

#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <ostream>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace odr
{

struct HeightOffset
{
    HeightOffset(double inner, double outer);

    double inner = 0;
    double outer = 0;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 9.5.4 Lane material
struct LaneMaterial
{
    LaneMaterial(std::string surface, std::optional<double> friction, std::optional<double> roughness);

    std::string           surface = "";
    std::optional<double> friction;
    std::optional<double> roughness;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 9.5.5 Lane speed limit
struct LaneSpeed
{
    LaneSpeed(double max, std::string unit);

    double      max = 0;
    std::string unit = "";

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 9.5.6 Lane access
struct LaneAccess
{
    LaneAccess(std::string rule, std::string restriction);

    std::string rule = "";        // "allow" or "deny"
    std::string restriction = ""; // e.g. "bus", "truck", "pedestrian"

    AdditionalData additional_data;
};

struct LaneKey
{
    LaneKey(std::string road_id, double lanesection_s0, int lane_id);
    std::string to_string() const;

    std::string road_id = "";
    double      lanesection_s0 = 0;
    int         lane_id = 0;

    bool operator==(const LaneKey& other) const
    {
        return this->road_id == other.road_id && this->lanesection_s0 == other.lanesection_s0 && this->lane_id == other.lane_id;
    }

    bool operator!=(const LaneKey& other) const
    {
        return !(*this == other);
    }
};

inline std::ostream& operator<<(std::ostream& os, const LaneKey& lk)
{
    return os << lk.to_string();
}

struct Lane
{
    Lane(std::string road_id, double lanesection_s0, int id, bool level, std::string type);

    std::vector<RoadMark> get_roadmarks(const double s_start, const double s_end) const;

    LaneKey     key;
    int         id;
    bool        level = false;
    std::string type = "";

    std::optional<int> predecessor; // first <predecessor> link, kept for compatibility
    std::optional<int> successor;   // first <successor> link, kept for compatibility
    // OpenDRIVE 1.6.1, 9.4 Lane linkage: a lane may have multiple predecessors / successors (lane splits and merges)
    std::vector<int> predecessors;
    std::vector<int> successors;

    CubicProfile lane_width;
    CubicProfile lane_border; // <border> elements as defined in file (OpenDRIVE >= 1.5), t-position of outer border
    CubicProfile outer_border;

    std::map<double, HeightOffset> s_to_height_offset;
    std::set<RoadMarkGroup>        roadmark_groups;

    // keyed by absolute s position (lane section s0 + sOffset), each record is valid until the next one or the end of the lane section
    std::map<double, LaneMaterial>     s_to_material;
    std::map<double, LaneSpeed>        s_to_speed;
    std::multimap<double, LaneAccess>  s_to_access; // several restrictions may start at the same s
    std::multimap<double, std::string> s_to_rule;   // OpenDRIVE 1.6.1, 9.7 Specific lane rules, free text

    AdditionalData additional_data;
};

} // namespace odr

namespace std
{
template<>
struct hash<odr::LaneKey>
{
    size_t operator()(const odr::LaneKey& key) const
    {
        return ((hash<string>()(key.road_id) ^ (hash<double>()(key.lanesection_s0) << 1)) >> 1) ^ (hash<int>()(key.lane_id) << 1);
    }
};

template<>
struct equal_to<odr::LaneKey>
{
    bool operator()(const odr::LaneKey& lhs, const odr::LaneKey& rhs) const
    {
        return (lhs.road_id == rhs.road_id) && (lhs.lanesection_s0 == rhs.lanesection_s0) && (lhs.lane_id == rhs.lane_id);
    }
};

template<>
struct less<odr::LaneKey>
{
    bool operator()(const odr::LaneKey& lhs, const odr::LaneKey& rhs) const
    {
        if (lhs.road_id != rhs.road_id)
            return lhs.road_id < rhs.road_id;
        if (lhs.lanesection_s0 != rhs.lanesection_s0)
            return lhs.lanesection_s0 < rhs.lanesection_s0;
        if (lhs.lane_id != rhs.lane_id)
            return lhs.lane_id < rhs.lane_id;
        return false;
    }
};
} // namespace std