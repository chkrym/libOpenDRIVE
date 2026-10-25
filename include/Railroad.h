#pragma once
#include "AdditionalData.h"
#include <optional>
#include <string>
#include <vector>

namespace odr
{

// OpenDRIVE 1.6.1, 13.2 Switches: <mainTrack> / <sideTrack>
struct RailroadSwitchTrack
{
    RailroadSwitchTrack() = default;
    RailroadSwitchTrack(std::string id, double s, std::string dir);

    std::string id = ""; // id of the track (road)
    double      s = 0;
    std::string dir = "+"; // "+" or "-"

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 13.2 Switches: <partner>
struct RailroadSwitchPartner
{
    RailroadSwitchPartner(std::string id, std::string name);

    std::string id = "";
    std::string name = "";

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 13.2 Switches, defined within <road><railroad>
struct RailroadSwitch
{
    RailroadSwitch(std::string road_id, std::string id, std::string name, std::string position);

    std::string road_id = "";
    std::string id = "";
    std::string name = "";
    std::string position = ""; // "dynamic" or "static"

    RailroadSwitchTrack                  main_track;
    RailroadSwitchTrack                  side_track;
    std::optional<RailroadSwitchPartner> partner;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 13.3 Stations: <segment>
struct StationPlatformSegment
{
    StationPlatformSegment(std::string road_id, double s_start, double s_end, std::string side);

    std::string road_id = "";
    double      s_start = 0;
    double      s_end = 0;
    std::string side = ""; // "left" or "right"

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 13.3 Stations: <platform>
struct StationPlatform
{
    StationPlatform(std::string id, std::string name);

    std::string id = "";
    std::string name = "";

    std::vector<StationPlatformSegment> segments;

    AdditionalData additional_data;
};

// OpenDRIVE 1.6.1, 13.3 Stations, defined directly within <OpenDRIVE>
struct Station
{
    Station(std::string id, std::string name, std::string type);

    std::string id = "";
    std::string name = "";
    std::string type = ""; // "small", "medium" or "large", optional

    std::vector<StationPlatform> platforms;

    AdditionalData additional_data;
};

} // namespace odr
