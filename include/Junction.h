#pragma once
#include "Utils.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace odr
{

struct JunctionLaneLink
{
    JunctionLaneLink(int from, int to);

    int from = 0;
    int to = 0;
};

} // namespace odr

namespace std
{
template<>
struct less<odr::JunctionLaneLink>
{
    bool operator()(const odr::JunctionLaneLink& lhs, const odr::JunctionLaneLink& rhs) const
    {
        return odr::compare_class_members(lhs, rhs, less<void>(), &odr::JunctionLaneLink::from, &odr::JunctionLaneLink::to);
    }
};
} // namespace std

namespace odr
{

// OpenDRIVE 1.6.1, 10.4.1 Virtual connections: predecessor / successor road of a virtual connection
struct JunctionConnectionLink
{
    JunctionConnectionLink(std::string element_type, std::string element_id);

    std::string                element_type = ""; // currently only "road" is allowed
    std::string                element_id = "";
    std::optional<double>      element_s;     // s-coordinate where the connection meets the road
    std::optional<std::string> element_dir;   // "+" or "-", relative to the s-direction of the road
    std::optional<std::string> contact_point; // "start" or "end"
};

struct JunctionConnection
{
    enum class ContactPoint
    {
        None,
        Start,
        End
    };

    JunctionConnection(std::string id, std::string incoming_road, std::string connecting_road, ContactPoint contact_point);

    std::string  id = "";
    std::string  incoming_road = "";
    std::string  connecting_road = "";
    ContactPoint contact_point = ContactPoint::None;
    std::string  type = "default"; // "default" or "virtual"

    std::optional<JunctionConnectionLink> predecessor; // only for virtual connections
    std::optional<JunctionConnectionLink> successor;   // only for virtual connections

    std::set<JunctionLaneLink> lane_links;
};

struct JunctionPriority
{
    JunctionPriority(std::string high, std::string low);

    std::string high = "";
    std::string low = "";
};

} // namespace odr

namespace std
{
template<>
struct less<odr::JunctionPriority>
{
    bool operator()(const odr::JunctionPriority& lhs, const odr::JunctionPriority& rhs) const
    {
        return odr::compare_class_members(lhs, rhs, less<void>(), &odr::JunctionPriority::high, &odr::JunctionPriority::low);
    }
};
} // namespace std

namespace odr
{

struct JunctionController
{
    JunctionController(std::string id, std::string type, std::uint32_t sequence);

    std::string   id = "";
    std::string   type = "";
    std::uint32_t sequence = 0;
};

class Junction
{
public:
    Junction(std::string name, std::string id);

    std::string name = "";
    std::string id = "";
    std::string type = "default"; // "default" or "virtual"

    std::map<std::string, JunctionConnection> id_to_connection;
    std::map<std::string, JunctionController> id_to_controller;
    std::set<JunctionPriority>                priorities;
};

// OpenDRIVE 1.6.1, 10.5 Junction groups, e.g. the junctions of one roundabout
struct JunctionGroup
{
    JunctionGroup(std::string name, std::string id, std::string type);

    std::string              name = "";
    std::string              id = "";
    std::string              type = ""; // "roundabout" or "unknown"
    std::vector<std::string> junction_ids;
};

} // namespace odr
