#include "OpenDriveMap.h"
#include "Geometries/Arc.h"
#include "Geometries/CubicSpline.h"
#include "Geometries/Line.h"
#include "Geometries/ParamPoly3.h"
#include "Geometries/RoadGeometry.h"
#include "Geometries/Spiral.h"
#include "Junction.h"
#include "Lane.h"
#include "LaneSection.h"
#include "LaneValidityRecord.h"
#include "Log.hpp"
#include "Math.hpp"
#include "RefLine.h"
#include "Road.h"
#include "RoadMark.h"
#include "RoadObject.h"
#include "RoadSignal.h"
#include "Utils.hpp"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iterator>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace odr
{

std::vector<LaneValidityRecord> extract_lane_validity_records(const pugi::xml_node& xml_node)
{
    std::vector<LaneValidityRecord> lane_validities;
    for (const auto& validity_node : xml_node.children("validity"))
    {
        LaneValidityRecord lane_validity{validity_node.attribute("fromLane").as_int(INT_MIN), validity_node.attribute("toLane").as_int(INT_MAX)};

        // fromLane should not be greater than toLane, since the standard defines them as follows:
        // fromLane - the minimum ID of lanes for which the object is valid
        // toLane - the maximum ID of lanes for which the object is valid
        // If we find such a violation, we set both IDs to 0 which means the
        // object is only applicable to the centerlane.
        odr::check_and_repair(
            lane_validity.from_lane <= lane_validity.to_lane,
            [&]()
            {
                lane_validity.from_lane = 0;
                lane_validity.to_lane = 0;
            },
            "Lane::validity::fromLane %d > Lane::validity::toLane %d, set to 0",
            lane_validity.from_lane,
            lane_validity.to_lane);

        lane_validities.push_back(std::move(lane_validity));
    }
    return lane_validities;
}

// OpenDRIVE 1.6.1, 5 Additional Data: direct <userData>, <include> and <dataQuality> children of xml_node
AdditionalData extract_additional_data(const pugi::xml_node& xml_node)
{
    AdditionalData additional_data;
    for (const pugi::xml_node user_data_node : xml_node.children("userData"))
    {
        UserData user_data(user_data_node.attribute("code").as_string(""), user_data_node.attribute("value").as_string(""));

        std::ostringstream content_stream;
        for (const pugi::xml_node child_node : user_data_node.children())
            child_node.print(content_stream, "", pugi::format_raw);
        user_data.content = content_stream.str();

        additional_data.user_data.push_back(user_data);
    }

    for (const pugi::xml_node include_node : xml_node.children("include"))
        additional_data.includes.push_back(include_node.attribute("file").as_string(""));

    if (const pugi::xml_node data_quality_node = xml_node.child("dataQuality"))
    {
        DataQuality data_quality;
        if (const pugi::xml_node error_node = data_quality_node.child("error"))
        {
            data_quality.error = DataQualityError(error_node.attribute("xyAbsolute").as_double(0.0),
                                                  error_node.attribute("zAbsolute").as_double(0.0),
                                                  error_node.attribute("xyRelative").as_double(0.0),
                                                  error_node.attribute("zRelative").as_double(0.0));
        }
        if (const pugi::xml_node raw_data_node = data_quality_node.child("rawData"))
        {
            data_quality.raw_data = DataQualityRawData(raw_data_node.attribute("date").as_string(""),
                                                       raw_data_node.attribute("source").as_string(""),
                                                       raw_data_node.attribute("sourceComment").as_string(""),
                                                       raw_data_node.attribute("postProcessing").as_string(""),
                                                       raw_data_node.attribute("postProcessingComment").as_string(""));
        }
        additional_data.data_quality = data_quality;
    }

    return additional_data;
}

// OpenDRIVE 1.6.1, 13.2: <mainTrack> / <sideTrack> of a railroad switch
RailroadSwitchTrack extract_switch_track(const pugi::xml_node& track_node)
{
    // the UML model names the direction attribute @dir, the 13.2 attribute table names it @position
    pugi::xml_attribute dir_attr = track_node.attribute("dir");
    if (!dir_attr)
        dir_attr = track_node.attribute("position");
    return RailroadSwitchTrack(track_node.attribute("id").as_string(""), track_node.attribute("s").as_double(0.0), dir_attr.as_string("+"));
}

// OpenDRIVE 1.6.1, 8.5 / 10.3: <CRG> elements of a <surface> element of a road or junction
std::vector<SurfaceCRG> extract_surface_crgs(const pugi::xml_node& surface_node)
{
    std::vector<SurfaceCRG> surface_crgs;
    for (const pugi::xml_node crg_node : surface_node.children("CRG"))
    {
        SurfaceCRG surface_crg(
            crg_node.attribute("file").as_string(""), crg_node.attribute("mode").as_string(""), crg_node.attribute("purpose").as_string("elevation"));
        if (const pugi::xml_attribute attr = crg_node.attribute("sStart"))
            surface_crg.s_start = attr.as_double();
        if (const pugi::xml_attribute attr = crg_node.attribute("sEnd"))
            surface_crg.s_end = attr.as_double();
        if (const pugi::xml_attribute attr = crg_node.attribute("orientation"))
            surface_crg.orientation = attr.as_string();
        surface_crg.s_offset = crg_node.attribute("sOffset").as_double(0.0);
        surface_crg.t_offset = crg_node.attribute("tOffset").as_double(0.0);
        surface_crg.z_offset = crg_node.attribute("zOffset").as_double(0.0);
        surface_crg.z_scale = crg_node.attribute("zScale").as_double(1.0);
        surface_crg.h_offset = crg_node.attribute("hOffset").as_double(0.0);
        surface_crgs.push_back(surface_crg);
    }
    return surface_crgs;
}

// OpenDRIVE 1.6.1, 7.6 Cubic polynom (deprecated): v(u) = a + b*u + c*u^2 + d*u^3 in the local u/v frame.
// The geometry @length is the arc length of the curve, so solve for the u at which that arc length is reached.
double get_poly3_u_end(const double b, const double c, const double d, const double length)
{
    const auto arc_length = [&](const double u_end)
    {
        // composite 3-point Gauss-Legendre quadrature of sqrt(1 + v'(u)^2)
        constexpr std::size_t num_intervals = 64;
        const double          sqrt_3_5 = std::sqrt(0.6);
        const double          h = u_end / num_intervals;
        double                sum = 0.0;
        for (std::size_t i = 0; i < num_intervals; i++)
        {
            const double u_mid = (i + 0.5) * h;
            for (const auto& [x, w] : {std::pair{-sqrt_3_5, 5.0 / 9.0}, std::pair{0.0, 8.0 / 9.0}, std::pair{sqrt_3_5, 5.0 / 9.0}})
            {
                const double u = u_mid + 0.5 * h * x;
                const double dv = b + 2 * c * u + 3 * d * u * u;
                sum += w * std::sqrt(1.0 + dv * dv);
            }
        }
        return 0.5 * h * sum;
    };

    // arc_length(u) >= u, so u_end is within [0, length]
    double u_lo = 0.0;
    double u_hi = length;
    for (std::size_t i = 0; i < 100 && (u_hi - u_lo) > 1e-12 * std::max(1.0, length); i++)
    {
        const double u_mid = 0.5 * (u_lo + u_hi);
        if (arc_length(u_mid) < length)
            u_lo = u_mid;
        else
            u_hi = u_mid;
    }
    return 0.5 * (u_lo + u_hi);
}

// collect the <include> elements below xml_node, skipping the free-form content of <userData>
void collect_include_nodes(const pugi::xml_node& xml_node, std::vector<pugi::xml_node>& include_nodes)
{
    for (const pugi::xml_node child_node : xml_node.children())
    {
        if (child_node.type() != pugi::node_element || std::string(child_node.name()) == "userData")
            continue;
        if (std::string(child_node.name()) == "include")
            include_nodes.push_back(child_node);
        else
            collect_include_nodes(child_node, include_nodes);
    }
}

// OpenDRIVE 1.6.1, 4.2 Combining Files: insert the content of every file referenced by an <include> element of xml_doc
// at the position of that element. Paths are relative to the including file. If the root element of the included file
// has the same name as the parent of the <include> element, its children are inserted, otherwise the root element itself.
// The <include> elements are kept, so they stay available as AdditionalData::includes.
void resolve_includes(pugi::xml_document& xml_doc, const std::filesystem::path& file_path, std::vector<std::filesystem::path>& file_stack)
{
    std::vector<pugi::xml_node> include_nodes;
    collect_include_nodes(xml_doc, include_nodes);

    for (pugi::xml_node include_node : include_nodes)
    {
        const std::string include_file = include_node.attribute("file").as_string("");
        if (include_file.empty())
            continue;

        const std::filesystem::path include_path = (file_path.parent_path() / std::filesystem::u8path(include_file)).lexically_normal();
        std::error_code             error_code;
        std::filesystem::path       canonical_path = std::filesystem::weakly_canonical(include_path, error_code);
        if (error_code)
            canonical_path = include_path;
        if (std::find(file_stack.begin(), file_stack.end(), canonical_path) != file_stack.end())
        {
            log::error("Recursive include of '%s' in '%s', skipped", include_file.c_str(), file_path.u8string().c_str());
            continue;
        }

        pugi::xml_document           include_doc;
        const pugi::xml_parse_result include_parse_result = include_doc.load_file(include_path.c_str());
        if (!include_parse_result)
        {
            log::warn(
                "Could not read include '%s' of '%s': %s", include_file.c_str(), file_path.u8string().c_str(), include_parse_result.description());
            continue;
        }

        file_stack.push_back(canonical_path);
        resolve_includes(include_doc, include_path, file_stack);
        file_stack.pop_back();

        pugi::xml_node       parent_node = include_node.parent();
        const pugi::xml_node include_root_node = include_doc.document_element();
        if (std::string(include_root_node.name()) == parent_node.name())
        {
            for (const pugi::xml_node child_node : include_root_node.children())
                parent_node.insert_copy_before(child_node, include_node);
        }
        else
        {
            parent_node.insert_copy_before(include_root_node, include_node);
        }
    }
}

OpenDriveMap::OpenDriveMap(const std::string& xodr_file,
                           const bool         center_map,
                           const bool         with_road_objects,
                           const bool         with_lateral_profile,
                           const bool         with_lane_height,
                           const bool         abs_z_for_for_local_road_obj_outline,
                           const bool         fix_spiral_edge_cases,
                           const bool         with_road_signals) :
    xodr_file(xodr_file)
{
    pugi::xml_document           xml_doc;
    const pugi::xml_parse_result xml_parse_result = xml_doc.load_file(xodr_file.c_str());
    if (!xml_parse_result)
        log::error("Error parsing xml: %s", xml_parse_result.description());

    std::vector<std::filesystem::path> file_stack;
    std::error_code                    error_code;
    const std::filesystem::path        canonical_xodr_path = std::filesystem::weakly_canonical(std::filesystem::u8path(xodr_file), error_code);
    file_stack.push_back(error_code ? std::filesystem::u8path(xodr_file) : canonical_xodr_path);
    resolve_includes(xml_doc, std::filesystem::u8path(xodr_file), file_stack);

    const pugi::xml_node odr_node = xml_doc.child("OpenDRIVE");

    const pugi::xml_node header_node = odr_node.child("header");
    this->header.rev_major = header_node.attribute("revMajor").as_uint(1);
    this->header.rev_minor = header_node.attribute("revMinor").as_uint(0);
    this->header.name = header_node.attribute("name").as_string("");
    this->header.version = header_node.attribute("version").as_string("");
    this->header.date = header_node.attribute("date").as_string("");
    this->header.vendor = header_node.attribute("vendor").as_string("");
    for (auto [attr_name, val_ptr] : {std::pair{"north", &(this->header.north)},
                                      std::pair{"south", &(this->header.south)},
                                      std::pair{"east", &(this->header.east)},
                                      std::pair{"west", &(this->header.west)}})
    {
        if (const pugi::xml_attribute attr = header_node.attribute(attr_name))
            *val_ptr = attr.as_double();
    }

    if (const pugi::xml_node offset_node = header_node.child("offset"))
    {
        this->header.offset = HeaderOffset(offset_node.attribute("x").as_double(0.0),
                                           offset_node.attribute("y").as_double(0.0),
                                           offset_node.attribute("z").as_double(0.0),
                                           offset_node.attribute("hdg").as_double(0.0));
    }

    if (const auto geoReference_node = header_node.child("geoReference"))
        this->proj4 = geoReference_node.text().as_string("");

    this->header.additional_data = extract_additional_data(header_node);
    this->additional_data = extract_additional_data(odr_node);

    std::size_t cnt = 1;
    if (center_map)
    {
        for (const pugi::xml_node road_node : odr_node.children("road"))
        {
            for (const pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
            {
                const double x0 = geometry_hdr_node.attribute("x").as_double(0.0);
                this->x_offs = this->x_offs + ((x0 - this->x_offs) / cnt);
                const double y0 = geometry_hdr_node.attribute("y").as_double(0.0);
                this->y_offs = this->y_offs + ((y0 - this->y_offs) / cnt);
                cnt++;
            }
        }
    }

    for (const pugi::xml_node junction_node : odr_node.children("junction"))
    {
        /* make junction */
        const std::string junction_id = junction_node.attribute("id").as_string("");

        Junction& junction =
            this->id_to_junction.insert({junction_id, Junction(junction_node.attribute("name").as_string(""), junction_id)}).first->second;
        junction.type = junction_node.attribute("type").as_string("default");
        junction.surface_crgs = extract_surface_crgs(junction_node.child("surface"));
        junction.additional_data = extract_additional_data(junction_node);

        for (const pugi::xml_node connection_node : junction_node.children("connection"))
        {
            const std::string connection_type = connection_node.attribute("type").as_string("default");
            const bool        is_virtual_connection = (connection_type == "virtual");

            // OpenDRIVE 1.6.1, 10.4.1: virtual connections are described by <predecessor> / <successor> instead of a connecting road
            std::string                      contact_point_str = connection_node.attribute("contactPoint").as_string("");
            JunctionConnection::ContactPoint junction_conn_contact_point = JunctionConnection::ContactPoint::None;
            if (!is_virtual_connection || !contact_point_str.empty())
            {
                odr::check_and_repair(
                    contact_point_str == "start" || contact_point_str == "end",
                    [&]() { contact_point_str = "start"; },
                    "Junction #%s: unknown connection::contactPoint '%s', set to 'start'",
                    junction_id.c_str(),
                    contact_point_str.c_str());
                junction_conn_contact_point =
                    (contact_point_str == "start") ? JunctionConnection::ContactPoint::Start : JunctionConnection::ContactPoint::End;
            }

            const std::string   junction_connection_id = connection_node.attribute("id").as_string("");
            JunctionConnection& junction_connection = junction.id_to_connection
                                                          .insert({junction_connection_id,
                                                                   JunctionConnection(junction_connection_id,
                                                                                      connection_node.attribute("incomingRoad").as_string(""),
                                                                                      connection_node.attribute("connectingRoad").as_string(""),
                                                                                      junction_conn_contact_point)})
                                                          .first->second;
            junction_connection.type = connection_type;

            for (const bool is_predecessor : {true, false})
            {
                const pugi::xml_node link_node = connection_node.child(is_predecessor ? "predecessor" : "successor");
                if (!link_node)
                    continue;

                JunctionConnectionLink link(link_node.attribute("elementType").as_string("road"), link_node.attribute("elementId").as_string(""));
                if (const pugi::xml_attribute attr = link_node.attribute("elementS"))
                    link.element_s = attr.as_double();
                if (const pugi::xml_attribute attr = link_node.attribute("elementDir"))
                    link.element_dir = attr.as_string();
                if (const pugi::xml_attribute attr = link_node.attribute("contactPoint"))
                    link.contact_point = attr.as_string();

                if (is_predecessor)
                    junction_connection.predecessor = link;
                else
                    junction_connection.successor = link;
            }

            for (const pugi::xml_node lane_link_node : connection_node.children("laneLink"))
            {
                const JunctionLaneLink lane_link(lane_link_node.attribute("from").as_int(0), lane_link_node.attribute("to").as_int(0));
                junction_connection.lane_links.insert(lane_link);
            }
        }

        const std::size_t num_conns = junction.id_to_connection.size();
        odr::check(num_conns > 0, "Junction #%s: 0 connections", junction_id.c_str());
        if (num_conns < 1)
            continue;

        for (const pugi::xml_node priority_node : junction_node.children("priority"))
        {
            const JunctionPriority junction_priority(priority_node.attribute("high").as_string(""), priority_node.attribute("low").as_string(""));
            junction.priorities.insert(junction_priority);
        }

        for (const pugi::xml_node controller_node : junction_node.children("controller"))
        {
            const std::string junction_controller_id = controller_node.attribute("id").as_string("");
            junction.id_to_controller.insert({junction_controller_id,
                                              JunctionController(junction_controller_id,
                                                                 controller_node.attribute("type").as_string(""),
                                                                 controller_node.attribute("sequence").as_uint(0))});
        }
    }

    for (const pugi::xml_node junction_group_node : odr_node.children("junctionGroup"))
    {
        const std::string junction_group_id = junction_group_node.attribute("id").as_string("");
        JunctionGroup&    junction_group = this->id_to_junction_group
                                            .insert({junction_group_id,
                                                     JunctionGroup(junction_group_node.attribute("name").as_string(""),
                                                                   junction_group_id,
                                                                   junction_group_node.attribute("type").as_string("unknown"))})
                                            .first->second;

        for (const pugi::xml_node junction_reference_node : junction_group_node.children("junctionReference"))
        {
            const std::string junction_id = junction_reference_node.attribute("junction").as_string("");
            odr::check(this->id_to_junction.find(junction_id) != this->id_to_junction.end(),
                       "JunctionGroup #%s: referenced junction #%s does not exist",
                       junction_group_id.c_str(),
                       junction_id.c_str());
            junction_group.junction_ids.push_back(junction_id);
        }
    }

    for (const pugi::xml_node controller_node : odr_node.children("controller"))
    {
        const std::string            controller_id = controller_node.attribute("id").as_string("");
        std::optional<std::uint32_t> sequence;
        if (const pugi::xml_attribute attr = controller_node.attribute("sequence"))
            sequence = attr.as_uint();

        Controller& controller =
            this->id_to_controller.insert({controller_id, Controller(controller_id, controller_node.attribute("name").as_string(""), sequence)})
                .first->second;
        for (const pugi::xml_node control_node : controller_node.children("control"))
        {
            controller.controls.emplace_back(control_node.attribute("signalId").as_string(""), control_node.attribute("type").as_string(""));
        }
    }

    for (const pugi::xml_node station_node : odr_node.children("station"))
    {
        const std::string station_id = station_node.attribute("id").as_string("");
        Station&          station =
            this->id_to_station
                .insert({station_id, Station(station_id, station_node.attribute("name").as_string(""), station_node.attribute("type").as_string(""))})
                .first->second;
        for (const pugi::xml_node platform_node : station_node.children("platform"))
        {
            StationPlatform platform(platform_node.attribute("id").as_string(""), platform_node.attribute("name").as_string(""));
            for (const pugi::xml_node segment_node : platform_node.children("segment"))
            {
                platform.segments.emplace_back(segment_node.attribute("roadId").as_string(""),
                                               segment_node.attribute("sStart").as_double(0.0),
                                               segment_node.attribute("sEnd").as_double(0.0),
                                               segment_node.attribute("side").as_string(""));
            }
            station.platforms.push_back(platform);
        }
    }

    odr::check(odr_node.child("road"), "No roads found");
    for (const pugi::xml_node road_node : odr_node.children("road"))
    {
        /* make road */
        std::string road_id = road_node.attribute("id").as_string("");
        odr::check_and_repair(
            this->id_to_road.find(road_id) == this->id_to_road.end(),
            [&]() { road_id = road_id + std::string("_dup"); },
            "Road #%s already exists, adding _dup suffix to id",
            road_id.c_str());

        std::string rule_str = std::string(road_node.attribute("rule").as_string("RHT"));
        std::transform(rule_str.begin(), rule_str.end(), rule_str.begin(), [](unsigned char c) { return std::tolower(c); });
        const bool is_left_hand_traffic = (rule_str == "lht");

        Road& road = this->id_to_road
                         .insert({road_id,
                                  Road(road_id,
                                       road_node.attribute("length").as_double(0.0),
                                       road_node.attribute("junction").as_string(""),
                                       road_node.attribute("name").as_string(""),
                                       is_left_hand_traffic)})
                         .first->second;

        odr::check_and_repair(
            road.length >= 0, [&]() { road.length = 0; }, "Road #%s: length %f < 0, set length = 0", road_id.c_str(), road.length);

        road.surface_crgs = extract_surface_crgs(road_node.child("surface"));
        /* parse railroad switches */
        for (const pugi::xml_node switch_node : road_node.child("railroad").children("switch"))
        {
            const std::string switch_id = switch_node.attribute("id").as_string("");
            RailroadSwitch    railroad_switch(
                road_id, switch_id, switch_node.attribute("name").as_string(""), switch_node.attribute("position").as_string(""));
            railroad_switch.main_track = extract_switch_track(switch_node.child("mainTrack"));
            railroad_switch.side_track = extract_switch_track(switch_node.child("sideTrack"));
            if (const pugi::xml_node partner_node = switch_node.child("partner"))
                railroad_switch.partner.emplace(partner_node.attribute("id").as_string(""), partner_node.attribute("name").as_string(""));
            road.id_to_switch.insert({switch_id, railroad_switch});
        }

        road.additional_data = extract_additional_data(road_node);

        /* parse road links */
        for (const bool is_predecessor : {true, false})
        {
            const pugi::xml_node road_link_node =
                is_predecessor ? road_node.child("link").child("predecessor") : road_node.child("link").child("successor");
            if (road_link_node)
            {
                RoadLink& link = is_predecessor ? road.predecessor : road.successor;
                link.id = road_link_node.attribute("elementId").as_string("");

                std::string type_str = road_link_node.attribute("elementType").as_string("");
                odr::check_and_repair(
                    type_str == "road" || type_str == "junction",
                    [&]() { type_str = "road"; },
                    "Road #%s: unknown link::%s::elementType '%s', set to 'road'",
                    road_id.c_str(),
                    is_predecessor ? "predecessor" : "successor",
                    type_str.c_str());
                link.type = (type_str == "road") ? RoadLink::Type::Road : RoadLink::Type::Junction;

                if (link.type == RoadLink::Type::Road)
                {
                    // junction connection has no contact point
                    std::string contact_point_str = road_link_node.attribute("contactPoint").as_string("");
                    odr::check_and_repair(
                        contact_point_str == "start" || contact_point_str == "end",
                        [&]() { contact_point_str = "start"; },
                        "Road #%s: unknown link::%s::contactPoint '%s', set to 'start'",
                        road_id.c_str(),
                        is_predecessor ? "predecessor" : "successor",
                        contact_point_str.c_str());
                    link.contact_point = (contact_point_str == "start") ? RoadLink::ContactPoint::Start : RoadLink::ContactPoint::End;
                }
            }
        }

        /* parse road neighbors */
        for (const pugi::xml_node road_neighbor_node : road_node.child("link").children("neighbor"))
        {
            const std::string road_neighbor_id = road_neighbor_node.attribute("elementId").as_string("");
            const std::string road_neighbor_side = road_neighbor_node.attribute("side").as_string("");
            const std::string road_neighbor_direction = road_neighbor_node.attribute("direction").as_string("");
            RoadNeighbor      road_neighbor(road_neighbor_id, road_neighbor_side, road_neighbor_direction);
            road.neighbors.push_back(road_neighbor);
        }

        /* parse road type and speed */
        for (const pugi::xml_node road_type_node : road_node.children("type"))
        {
            double            s = road_type_node.attribute("s").as_double(0.0);
            const std::string type = road_type_node.attribute("type").as_string("");

            odr::check_and_repair(
                s >= 0, [&]() { s = 0; }, "Road #%s: type record '%s' s %f < 0, set s=0", road_id.c_str(), type.c_str(), s);

            road.s_to_type[s] = type;
            if (const pugi::xml_attribute country_attr = road_type_node.attribute("country"))
                road.s_to_country[s] = country_attr.as_string("");
            if (const pugi::xml_node node = road_type_node.child("speed"))
            {
                const std::string speed_record_max = node.attribute("max").as_string("");
                const std::string speed_record_unit = node.attribute("unit").as_string("m/s");
                SpeedRecord       speed_record(speed_record_max, speed_record_unit);
                road.s_to_speed.insert({s, speed_record});
            }
        }

        /* make ref_line - parse road geometries */
        for (const pugi::xml_node geometry_hdr_node : road_node.child("planView").children("geometry"))
        {
            double       s0 = geometry_hdr_node.attribute("s").as_double(0.0);
            const double x0 = geometry_hdr_node.attribute("x").as_double(0.0) - this->x_offs;
            const double y0 = geometry_hdr_node.attribute("y").as_double(0.0) - this->y_offs;
            const double hdg0 = geometry_hdr_node.attribute("hdg").as_double(0.0);
            double       length = geometry_hdr_node.attribute("length").as_double(0.0);

            odr::check_and_repair(
                s0 >= 0, [&]() { s0 = 0; }, "Road #%s: planView::geometry::s %f < 0, set s=0", road_id.c_str(), s0);
            odr::check_and_repair(
                length >= 0, [&]() { length = 0; }, "Road #%s: planView::geometry::length %f < 0, set length=0", road_id.c_str(), length);

            const pugi::xml_node geometry_node = geometry_hdr_node.first_child();
            const std::string    geometry_type = geometry_node.name();
            if (geometry_type == "line")
            {
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
            }
            else if (geometry_type == "spiral")
            {
                const double curv_start = geometry_node.attribute("curvStart").as_double(0.0);
                const double curv_end = geometry_node.attribute("curvEnd").as_double(0.0);
                if (!fix_spiral_edge_cases)
                {
                    road.ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                }
                else
                {
                    if (std::abs(curv_start) < 1e-6 && std::abs(curv_end) < 1e-6)
                    {
                        // In effect a line
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Line>(s0, x0, y0, hdg0, length);
                    }
                    else if (std::abs(curv_end - curv_start) < 1e-6)
                    {
                        // In effect an arc
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curv_start);
                    }
                    else
                    {
                        // True spiral
                        road.ref_line.s0_to_geometry[s0] = std::make_unique<Spiral>(s0, x0, y0, hdg0, length, curv_start, curv_end);
                    }
                }
            }
            else if (geometry_type == "arc")
            {
                const double curvature = geometry_node.attribute("curvature").as_double(0.0);
                road.ref_line.s0_to_geometry[s0] = std::make_unique<Arc>(s0, x0, y0, hdg0, length, curvature);
            }
            else if (geometry_type == "poly3")
            {
                // @a only shifts the u/v origin along the v-axis and does not affect the curve position, so it is not read
                const double b = geometry_node.attribute("b").as_double(0.0);
                const double c = geometry_node.attribute("c").as_double(0.0);
                const double d = geometry_node.attribute("d").as_double(0.0);

                // OpenDRIVE 1.6.1, 7.6.2: (@x,@y) is located at u=0 on the v-axis, @a shifts the u/v origin along the v-axis and
                // arctan(@b) is the start heading of the curve w.r.t. the u/v system. So the curve starts at (@x,@y) with heading @hdg:
                // shift by -a and rotate the u/v system by theta = -arctan(b) relative to @hdg.
                const double u_end = get_poly3_u_end(b, c, d, length);
                const double theta = -std::atan(b);
                const double cos_theta = std::cos(theta);
                const double sin_theta = std::sin(theta);

                // express as normalized paramPoly3 with p in [0, 1], u = u_end * p:
                // U(p) = u * cos(theta) - (v(u) - a) * sin(theta), V(p) = u * sin(theta) + (v(u) - a) * cos(theta)
                const double bU = u_end * (cos_theta - b * sin_theta);
                const double cU = -c * u_end * u_end * sin_theta;
                const double dU = -d * u_end * u_end * u_end * sin_theta;
                const double bV = u_end * (sin_theta + b * cos_theta);
                const double cV = c * u_end * u_end * cos_theta;
                const double dV = d * u_end * u_end * u_end * cos_theta;
                road.ref_line.s0_to_geometry[s0] = std::make_unique<ParamPoly3>(s0, x0, y0, hdg0, length, 0.0, bU, cU, dU, 0.0, bV, cV, dV, true);
            }
            else if (geometry_type == "paramPoly3")
            {
                const double aU = geometry_node.attribute("aU").as_double(0.0);
                const double bU = geometry_node.attribute("bU").as_double(0.0);
                const double cU = geometry_node.attribute("cU").as_double(0.0);
                const double dU = geometry_node.attribute("dU").as_double(0.0);
                const double aV = geometry_node.attribute("aV").as_double(0.0);
                const double bV = geometry_node.attribute("bV").as_double(0.0);
                const double cV = geometry_node.attribute("cV").as_double(0.0);
                const double dV = geometry_node.attribute("dV").as_double(0.0);

                bool pRange_normalized = true;
                if (geometry_node.attribute("pRange") || geometry_hdr_node.attribute("pRange"))
                {
                    std::string pRange_str = geometry_node.attribute("pRange") ? geometry_node.attribute("pRange").as_string("")
                                                                               : geometry_hdr_node.attribute("pRange").as_string("");
                    std::transform(pRange_str.begin(), pRange_str.end(), pRange_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (pRange_str == "arclength")
                        pRange_normalized = false;
                }
                road.ref_line.s0_to_geometry[s0] =
                    std::make_unique<ParamPoly3>(s0, x0, y0, hdg0, length, aU, bU, cU, dU, aV, bV, cV, dV, pRange_normalized);
            }
            else
            {
                log::error("Could not parse %s", geometry_type.c_str());
                continue;
            }
        }

        std::map<std::string /*x path query*/, CubicProfile&> cubic_profile_fields{
            {".//elevationProfile//elevation", road.ref_line.elevation_profile}, {".//lanes//laneOffset", road.lane_offset}};

        if (with_lateral_profile)
            cubic_profile_fields.insert({".//lateralProfile//superelevation", road.superelevation});

        /* parse elevation profiles, lane offsets, superelevation */
        for (auto& [xpath_query_str, cubic_profile] : cubic_profile_fields)
        {
            /* handle splines not starting at s=0, assume value 0 until start */
            cubic_profile.segments[0.0] = CubicPoly(0.0, 0.0, 0.0, 0.0);

            pugi::xpath_node_set nodes = road_node.select_nodes(xpath_query_str.c_str());
            for (pugi::xpath_node node : nodes)
            {
                double       s0 = node.node().attribute("s").as_double(0.0);
                const double a = node.node().attribute("a").as_double(0.0);
                const double b = node.node().attribute("b").as_double(0.0);
                const double c = node.node().attribute("c").as_double(0.0);
                const double d = node.node().attribute("d").as_double(0.0);

                odr::check_and_repair(
                    s0 >= 0, [&]() { s0 = 0; }, "Road #%s: %s s %f < 0, set s=0", road_id.c_str(), xpath_query_str.c_str(), s0);

                cubic_profile.segments[s0] = CubicPoly(a, b, c, d, s0);
            }
        }

        /* parse crossfall - has extra attribute side */
        if (with_lateral_profile)
        {
            for (pugi::xml_node crossfall_node : road_node.child("lateralProfile").children("crossfall"))
            {
                double       s0 = crossfall_node.attribute("s").as_double(0.0);
                const double a = crossfall_node.attribute("a").as_double(0.0);
                const double b = crossfall_node.attribute("b").as_double(0.0);
                const double c = crossfall_node.attribute("c").as_double(0.0);
                const double d = crossfall_node.attribute("d").as_double(0.0);

                odr::check_and_repair(
                    s0 >= 0, [&]() { s0 = 0; }, "Road #%s: lateralProfile::crossfall::s %f < 0, set s=0", road_id.c_str(), s0);

                CubicPoly crossfall_poly(a, b, c, d, s0);
                road.crossfall.segments[s0] = crossfall_poly;
                if (const pugi::xml_attribute side = crossfall_node.attribute("side"))
                {
                    std::string side_str = side.as_string("");
                    std::transform(side_str.begin(), side_str.end(), side_str.begin(), [](unsigned char c) { return std::tolower(c); });
                    if (side_str == "left")
                        road.crossfall.sides[s0] = Crossfall::Side::Left;
                    else if (side_str == "right")
                        road.crossfall.sides[s0] = Crossfall::Side::Right;
                    else
                        road.crossfall.sides[s0] = Crossfall::Side::Both;
                }
            }

            /* parse lateralProfile shape - polynomials over t at given s positions */
            for (const pugi::xml_node shape_node : road_node.child("lateralProfile").children("shape"))
            {
                double       s0 = shape_node.attribute("s").as_double(0.0);
                const double t0 = shape_node.attribute("t").as_double(0.0);
                const double a = shape_node.attribute("a").as_double(0.0);
                const double b = shape_node.attribute("b").as_double(0.0);
                const double c = shape_node.attribute("c").as_double(0.0);
                const double d = shape_node.attribute("d").as_double(0.0);

                odr::check_and_repair(
                    s0 >= 0, [&]() { s0 = 0; }, "Road #%s: lateralProfile::shape::s %f < 0, set s=0", road_id.c_str(), s0);

                road.s_to_lateral_shape[s0].segments[t0] = CubicPoly(a, b, c, d, t0);
            }
        }

        /* parse road lane sections and lanes */
        for (const pugi::xml_node lanesection_node : road_node.child("lanes").children("laneSection"))
        {
            const double s0 = lanesection_node.attribute("s").as_double(0.0);
            LaneSection& lanesection = road.s_to_lanesection.insert({s0, LaneSection(road_id, s0)}).first->second;
            lanesection.additional_data = extract_additional_data(lanesection_node);

            for (const pugi::xpath_node lane_xpath_node : lanesection_node.select_nodes(".//lane"))
            {
                const pugi::xml_node lane_node = lane_xpath_node.node();
                const int            lane_id = lane_node.attribute("id").as_int(0);

                Lane& lane =
                    lanesection.id_to_lane
                        .insert({lane_id,
                                 Lane(road_id, s0, lane_id, lane_node.attribute("level").as_bool(false), lane_node.attribute("type").as_string(""))})
                        .first->second;

                lane.additional_data = extract_additional_data(lane_node);

                // OpenDRIVE 1.6.1, 9.4 Lane linkage: "Multiple predecessors and successors shall be used" for lane splits and merges
                for (const pugi::xml_node predecessor_node : lane_node.child("link").children("predecessor"))
                {
                    if (const pugi::xml_attribute id_attr = predecessor_node.attribute("id"))
                        lane.predecessors.push_back(id_attr.as_int());
                }
                for (const pugi::xml_node successor_node : lane_node.child("link").children("successor"))
                {
                    if (const pugi::xml_attribute id_attr = successor_node.attribute("id"))
                        lane.successors.push_back(id_attr.as_int());
                }
                if (!lane.predecessors.empty())
                    lane.predecessor = lane.predecessors.front();
                if (!lane.successors.empty())
                    lane.successor = lane.successors.front();

                for (const pugi::xml_node lane_width_node : lane_node.children("width"))
                {
                    double       s_offset = lane_width_node.attribute("sOffset").as_double(0.0);
                    const double a = lane_width_node.attribute("a").as_double(0.0);
                    const double b = lane_width_node.attribute("b").as_double(0.0);
                    const double c = lane_width_node.attribute("c").as_double(0.0);
                    const double d = lane_width_node.attribute("d").as_double(0.0);

                    odr::check_and_repair(
                        s_offset >= 0,
                        [&]() { s_offset = 0; },
                        "Road #%s LaneSection %f Lane #%d: width::sOffset %f < 0, set sOffset=0",
                        road_id.c_str(),
                        s0,
                        lane_id,
                        s_offset);
                    CubicPoly width_poly(a, b, c, d, s0 + s_offset);

                    // OpenDRIVE Format Specification, Rev. 1.4, 3.3.1 General:
                    // "The reference line itself is defined as lane zero and must not have a width entry (i.e. its width must always be 0.0)."
                    if (lane_id == 0)
                        odr::check_and_repair(
                            width_poly.is_zero(),
                            [&]() { width_poly.set_zero(); },
                            "Road #%s LaneSection %f Lane #0: width must be 0, set to 0",
                            road_id.c_str(),
                            s0);

                    lane.lane_width.segments[s0 + s_offset] = width_poly;
                }

                /* parse lane material, speed, access and rule records */
                const auto get_record_s0 = [&](const pugi::xml_node& node, const char* record_name)
                {
                    double s_offset = node.attribute("sOffset").as_double(0.0);
                    odr::check_and_repair(
                        s_offset >= 0,
                        [&]() { s_offset = 0; },
                        "Road #%s LaneSection %f Lane #%d: %s::sOffset %f < 0, set sOffset=0",
                        road_id.c_str(),
                        s0,
                        lane_id,
                        record_name,
                        s_offset);
                    return s0 + s_offset;
                };

                const auto get_optional_double = [](const pugi::xml_node& node, const char* attr_name) -> std::optional<double>
                {
                    if (const pugi::xml_attribute attr = node.attribute(attr_name))
                        return attr.as_double();
                    return std::nullopt;
                };

                for (const pugi::xml_node material_node : lane_node.children("material"))
                {
                    lane.s_to_material.insert_or_assign(get_record_s0(material_node, "material"),
                                                        LaneMaterial(material_node.attribute("surface").as_string(""),
                                                                     get_optional_double(material_node, "friction"),
                                                                     get_optional_double(material_node, "roughness")));
                }

                for (const pugi::xml_node speed_node : lane_node.children("speed"))
                {
                    lane.s_to_speed.insert_or_assign(
                        get_record_s0(speed_node, "speed"),
                        LaneSpeed(speed_node.attribute("max").as_double(0.0), speed_node.attribute("unit").as_string("m/s")));
                }

                for (const pugi::xml_node access_node : lane_node.children("access"))
                {
                    lane.s_to_access.emplace(
                        get_record_s0(access_node, "access"),
                        LaneAccess(access_node.attribute("rule").as_string(""), access_node.attribute("restriction").as_string("")));
                }

                for (const pugi::xml_node rule_node : lane_node.children("rule"))
                    lane.s_to_rule.emplace(get_record_s0(rule_node, "rule"), rule_node.attribute("value").as_string(""));

                for (const pugi::xml_node lane_border_node : lane_node.children("border"))
                {
                    double       s_offset = lane_border_node.attribute("sOffset").as_double(0.0);
                    const double a = lane_border_node.attribute("a").as_double(0.0);
                    const double b = lane_border_node.attribute("b").as_double(0.0);
                    const double c = lane_border_node.attribute("c").as_double(0.0);
                    const double d = lane_border_node.attribute("d").as_double(0.0);

                    odr::check_and_repair(
                        s_offset >= 0,
                        [&]() { s_offset = 0; },
                        "Road #%s LaneSection %f Lane #%d: border::sOffset %f < 0, set sOffset=0",
                        road_id.c_str(),
                        s0,
                        lane_id,
                        s_offset);

                    lane.lane_border.segments[s0 + s_offset] = CubicPoly(a, b, c, d, s0 + s_offset);
                }

                if (with_lane_height)
                {
                    for (const pugi::xml_node lane_height_node : lane_node.children("height"))
                    {
                        double       s_offset = lane_height_node.attribute("sOffset").as_double(0.0);
                        const double inner = lane_height_node.attribute("inner").as_double(0.0);
                        const double outer = lane_height_node.attribute("outer").as_double(0.0);

                        odr::check_and_repair(
                            s_offset >= 0,
                            [&]() { s_offset = 0; },
                            "Road #%s LaneSection %f Lane #%d: height::sOffset %f < 0, set sOffset=0",
                            road_id.c_str(),
                            s0,
                            lane_id,
                            s_offset);
                        lane.s_to_height_offset.insert({s0 + s_offset, HeightOffset(inner, outer)});
                    }
                }

                for (const pugi::xml_node roadmark_node : lane_node.children("roadMark"))
                {
                    RoadMarkGroup roadmark_group(road_id,
                                                 s0,
                                                 lane_id,
                                                 roadmark_node.attribute("width").as_double(-1),
                                                 roadmark_node.attribute("height").as_double(0),
                                                 roadmark_node.attribute("sOffset").as_double(0),
                                                 roadmark_node.attribute("type").as_string("none"),
                                                 roadmark_node.attribute("weight").as_string("standard"),
                                                 roadmark_node.attribute("color").as_string("standard"),
                                                 roadmark_node.attribute("material").as_string("standard"),
                                                 roadmark_node.attribute("laneChange").as_string("both"));

                    odr::check_and_repair(
                        roadmark_group.s_offset >= 0,
                        [&]() { roadmark_group.s_offset = 0; },
                        "Road #%s LaneSection %f Lane #%d: roadMark::sOffset %f < 0, set sOffset=0",
                        road_id.c_str(),
                        s0,
                        lane_id,
                        roadmark_group.s_offset);
                    const double roadmark_group_s0 = s0 + roadmark_group.s_offset;

                    if (const pugi::xml_node roadmark_type_node = roadmark_node.child("type"))
                    {
                        const std::string name = roadmark_type_node.attribute("name").as_string("");
                        const double      line_width_1 = roadmark_type_node.attribute("width").as_double(-1);

                        for (const pugi::xml_node roadmarks_line_node : roadmark_type_node.children("line"))
                        {
                            const double line_width_0 = roadmarks_line_node.attribute("width").as_double(-1);
                            const double roadmark_width = line_width_0 < 0 ? line_width_1 : line_width_0;

                            RoadMarksLine roadmarks_line(road_id,
                                                         s0,
                                                         lane_id,
                                                         roadmark_group_s0,
                                                         roadmark_width,
                                                         roadmarks_line_node.attribute("length").as_double(0),
                                                         roadmarks_line_node.attribute("space").as_double(0),
                                                         roadmarks_line_node.attribute("tOffset").as_double(0),
                                                         roadmarks_line_node.attribute("sOffset").as_double(0),
                                                         name,
                                                         roadmarks_line_node.attribute("rule").as_string("none"),
                                                         roadmarks_line_node.attribute("color").as_string(roadmark_group.color.c_str()));

                            for (auto [val_name, val_ptr] : {std::pair{"length", &(roadmarks_line.length)},
                                                             std::pair{"space", &(roadmarks_line.space)},
                                                             std::pair{"s_offset", &(roadmarks_line.s_offset)}})
                            {
                                odr::check_and_repair((*val_ptr) >= 0,
                                                      [p = val_ptr]() { *p = 0; },
                                                      "Road #%s LaneSection %f Lane #%d RoadMark %f: type::line::%s %f < 0, set %s=0",
                                                      road_id.c_str(),
                                                      s0,
                                                      lane_id,
                                                      roadmark_group_s0,
                                                      val_name,
                                                      *val_ptr,
                                                      val_name);
                            }

                            roadmark_group.roadmark_lines.emplace(std::move(roadmarks_line));
                        }
                    }

                    if (const pugi::xml_node roadmark_explicit_node = roadmark_node.child("explicit"))
                    {
                        for (const pugi::xml_node roadmarks_line_node : roadmark_explicit_node.children("line"))
                        {
                            RoadMarksLine roadmarks_line(road_id,
                                                         s0,
                                                         lane_id,
                                                         roadmark_group_s0,
                                                         roadmarks_line_node.attribute("width").as_double(-1),
                                                         roadmarks_line_node.attribute("length").as_double(0),
                                                         0.0,
                                                         roadmarks_line_node.attribute("tOffset").as_double(0),
                                                         roadmarks_line_node.attribute("sOffset").as_double(0),
                                                         "",
                                                         roadmarks_line_node.attribute("rule").as_string("none"),
                                                         roadmark_group.color);

                            for (auto [val_name, val_ptr] :
                                 {std::pair{"length", &(roadmarks_line.length)}, std::pair{"s_offset", &(roadmarks_line.s_offset)}})
                            {
                                odr::check_and_repair((*val_ptr) >= 0,
                                                      [p = val_ptr]() { *p = 0; },
                                                      "Road #%s LaneSection %f Lane #%d RoadMark %f: explicit::line::%s %f < 0, set %s=0",
                                                      road_id.c_str(),
                                                      s0,
                                                      lane_id,
                                                      roadmark_group_s0,
                                                      val_name,
                                                      *val_ptr,
                                                      val_name);
                            }

                            roadmark_group.roadmark_explicit_lines.emplace(std::move(roadmarks_line));
                        }
                    }

                    for (const pugi::xml_node sway_node : roadmark_node.children("sway"))
                    {
                        double ds = sway_node.attribute("ds").as_double(0.0);
                        odr::check_and_repair(
                            ds >= 0,
                            [&]() { ds = 0; },
                            "Road #%s LaneSection %f Lane #%d RoadMark %f: sway::ds %f < 0, set ds=0",
                            road_id.c_str(),
                            s0,
                            lane_id,
                            roadmark_group_s0,
                            ds);
                        roadmark_group.sway.segments[roadmark_group_s0 + ds] = CubicPoly(sway_node.attribute("a").as_double(0.0),
                                                                                         sway_node.attribute("b").as_double(0.0),
                                                                                         sway_node.attribute("c").as_double(0.0),
                                                                                         sway_node.attribute("d").as_double(0.0),
                                                                                         roadmark_group_s0 + ds);
                    }

                    lane.roadmark_groups.emplace(std::move(roadmark_group));
                }
            }

            // OpenDRIVE 1.6.1, 9.2 Lane sections: a lane section with @singleSide="true" is only valid for the side(s) it defines.
            // The lanes of the other side continue from the preceding lane section, so they are copied into this one.
            if (lanesection_node.attribute("singleSide").as_bool(false) && road.s_to_lanesection.begin()->first < s0)
            {
                LaneSection& prev_lanesection = std::prev(road.s_to_lanesection.find(s0))->second;

                bool has_left = false;
                bool has_right = false;
                for (const auto& [id, lane] : lanesection.id_to_lane)
                {
                    has_left = has_left || id > 0;
                    has_right = has_right || id < 0;
                }

                for (auto& [id, prev_lane] : prev_lanesection.id_to_lane)
                {
                    if (id == 0 || (id > 0 && has_left) || (id < 0 && has_right))
                        continue;

                    Lane lane = prev_lane;
                    lane.key.lanesection_s0 = s0;
                    lane.predecessor = id;
                    lane.predecessors = {id};
                    lane.successor = prev_lane.successor;
                    lane.successors = prev_lane.successors;
                    prev_lane.successor = id;
                    prev_lane.successors = {id};
                    lanesection.id_to_lane.insert({id, lane});
                }
            }

            /* derive lane borders from lane widths */
            const auto id_lane_iter0 = lanesection.id_to_lane.find(0);
            if (id_lane_iter0 == lanesection.id_to_lane.end())
                throw std::runtime_error("lane section does not have lane #0");

            // OpenDRIVE 1.6.1, 9.5.2 Lane borders:
            // "Lane width and lane border elements are mutually exclusive within the same lane group. If both width and lane border
            // elements are present for a lane section in the OpenDRIVE file, the application shall use the information from the <width>
            // elements."
            const auto group_uses_borders = [](auto begin, auto end)
            {
                bool has_border = false;
                for (auto iter = begin; iter != end; iter++)
                {
                    if (!iter->second.lane_width.segments.empty())
                        return false;
                    has_border = has_border || !iter->second.lane_border.segments.empty();
                }
                return has_border;
            };

            /* iterate from id #1 towards +inf */
            const auto id_lane_iter1 = std::next(id_lane_iter0);
            const bool left_uses_borders = group_uses_borders(id_lane_iter1, lanesection.id_to_lane.end());
            for (auto iter = id_lane_iter1; iter != lanesection.id_to_lane.end(); iter++)
            {
                const CubicProfile inner_border = (iter == id_lane_iter1) ? CubicProfile() : std::prev(iter)->second.outer_border;
                if (left_uses_borders)
                {
                    iter->second.outer_border = iter->second.lane_border.segments.empty() ? inner_border : iter->second.lane_border;
                    iter->second.lane_width = iter->second.outer_border.add(inner_border.negate());
                }
                else
                {
                    iter->second.outer_border = inner_border.add(iter->second.lane_width);
                }
            }

            /* iterate from id #-1 towards -inf */
            // "For a reverse iterator r constructed from an iterator i, the relationship &*r == &*(i - 1) is always true"
            // The reverse iterator points to the element that is one before the element referred by the id_lane_iter0!
            const std::map<int, Lane>::reverse_iterator r_id_lane_iter1(id_lane_iter0);
            const bool                                  right_uses_borders = group_uses_borders(r_id_lane_iter1, lanesection.id_to_lane.rend());
            for (auto r_iter = r_id_lane_iter1; r_iter != lanesection.id_to_lane.rend(); r_iter++)
            {
                const CubicProfile inner_border = (r_iter == r_id_lane_iter1) ? CubicProfile() : std::prev(r_iter)->second.outer_border;
                if (right_uses_borders)
                {
                    r_iter->second.outer_border = r_iter->second.lane_border.segments.empty() ? inner_border : r_iter->second.lane_border;
                    r_iter->second.lane_width = inner_border.add(r_iter->second.outer_border.negate());
                }
                else
                {
                    r_iter->second.outer_border = inner_border.add(r_iter->second.lane_width.negate());
                }
            }

            // OpenDRIVE® Format Specification, Rev. 1.4, 3.3.2 Lane Offset:
            // "... lane 0 may be offset using a cubic polynom"
            // Lane borders are absolute t-positions; OpenDRIVE 1.6.1, 9.5.2: "Border elements shall not exist together with lane offset."
            for (auto& id_lane : lanesection.id_to_lane)
            {
                const bool uses_borders = (id_lane.first > 0 && left_uses_borders) || (id_lane.first < 0 && right_uses_borders);
                if (!uses_borders)
                    id_lane.second.outer_border = id_lane.second.outer_border.add(road.lane_offset);
            }
        }

        /* parse road objects */
        if (with_road_objects)
        {
            const RoadObjectCorner::Type default_local_outline_type =
                abs_z_for_for_local_road_obj_outline ? RoadObjectCorner::Type::Local_AbsZ : RoadObjectCorner::Type::Local_RelZ;

            for (pugi::xml_node object_node : road_node.child("objects").children("object"))
            {
                std::string road_object_id = object_node.attribute("id").as_string("");
                odr::check_and_repair(
                    road.id_to_object.find(road_object_id) == road.id_to_object.end(),
                    [&]() { road_object_id = road_object_id + std::string("_dup"); },
                    "Road #%s Object #%s already exists, adding _dup suffix to id",
                    road_id.c_str(),
                    road_object_id.c_str());

                const bool  is_dynamic_object = std::string(object_node.attribute("dynamic").as_string("no")) == "yes" ? true : false;
                RoadObject& road_object = road.id_to_object
                                              .insert({road_object_id,
                                                       RoadObject(road_id,
                                                                  road_object_id,
                                                                  object_node.attribute("s").as_double(0),
                                                                  object_node.attribute("t").as_double(0),
                                                                  object_node.attribute("zOffset").as_double(0),
                                                                  object_node.attribute("length").as_double(0),
                                                                  object_node.attribute("validLength").as_double(0),
                                                                  object_node.attribute("width").as_double(0),
                                                                  object_node.attribute("radius").as_double(0),
                                                                  object_node.attribute("height").as_double(0),
                                                                  object_node.attribute("hdg").as_double(0),
                                                                  object_node.attribute("pitch").as_double(0),
                                                                  object_node.attribute("roll").as_double(0),
                                                                  object_node.attribute("type").as_string(""),
                                                                  object_node.attribute("name").as_string(""),
                                                                  object_node.attribute("orientation").as_string(""),
                                                                  object_node.attribute("subtype").as_string(""),
                                                                  is_dynamic_object)})
                                              .first->second;

                for (auto [val_name, val_ptr] : {std::pair{"s0", &(road_object.s0)},
                                                 std::pair{"valid_length", &(road_object.valid_length)},
                                                 std::pair{"length", &(road_object.length)},
                                                 std::pair{"width", &(road_object.width)},
                                                 std::pair{"radius", &(road_object.radius)}})
                {
                    odr::check_and_repair((*val_ptr) >= 0,
                                          [p = val_ptr]() { *p = 0; },
                                          "Road #%s: object::%s %f < 0, set %s=0",
                                          road_id.c_str(),
                                          val_name,
                                          *val_ptr,
                                          val_name);
                }

                for (pugi::xml_node repeat_node : object_node.children("repeat"))
                {
                    RoadObjectRepeat road_object_repeat(repeat_node.attribute("s").as_double(NAN),
                                                        repeat_node.attribute("length").as_double(0),
                                                        repeat_node.attribute("distance").as_double(0),
                                                        repeat_node.attribute("tStart").as_double(NAN),
                                                        repeat_node.attribute("tEnd").as_double(NAN),
                                                        repeat_node.attribute("widthStart").as_double(NAN),
                                                        repeat_node.attribute("widthEnd").as_double(NAN),
                                                        repeat_node.attribute("heightStart").as_double(NAN),
                                                        repeat_node.attribute("heightEnd").as_double(NAN),
                                                        repeat_node.attribute("zOffsetStart").as_double(NAN),
                                                        repeat_node.attribute("zOffsetEnd").as_double(NAN),
                                                        repeat_node.attribute("lengthStart").as_double(NAN),
                                                        repeat_node.attribute("lengthEnd").as_double(NAN),
                                                        repeat_node.attribute("radiusStart").as_double(NAN),
                                                        repeat_node.attribute("radiusEnd").as_double(NAN));

                    for (auto [val_name, val_ptr] : {std::pair{"s0", &(road_object_repeat.s0)},
                                                     std::pair{"width_start", &(road_object_repeat.width_start)},
                                                     std::pair{"width_end", &(road_object_repeat.width_end)},
                                                     std::pair{"length_start", &(road_object_repeat.length_start)},
                                                     std::pair{"length_end", &(road_object_repeat.length_end)},
                                                     std::pair{"radius_start", &(road_object_repeat.radius_start)},
                                                     std::pair{"radius_end", &(road_object_repeat.radius_end)}})
                    {
                        odr::check_and_repair(
                            std::isnan((*val_ptr)) || (*val_ptr) >= 0,
                            [p = val_ptr]() { *p = 0; },
                            "Road #%s Object #%s: repeat::%s %f < 0, set %s=0",
                            road_id.c_str(),
                            road_object_id.c_str(),
                            val_name,
                            *val_ptr,
                            val_name);
                    }
                    for (auto [val_name, val_ptr] :
                         {std::pair{"length", &(road_object_repeat.length)}, std::pair{"distance", &(road_object_repeat.distance)}})
                    {
                        odr::check_and_repair((*val_ptr) >= 0,
                                              [p = val_ptr]() { *p = 0; },
                                              "Road #%s Object #%s: repeat::%s %f < 0, set %s=0",
                                              road_id.c_str(),
                                              road_object_id.c_str(),
                                              val_name,
                                              *val_ptr,
                                              val_name);
                    }

                    road_object.repeats.push_back(road_object_repeat);
                }

                /* since v1.45 multiple <outline> are allowed and parent tag is <outlines>, not <object>; this supports v1.4 and v1.45+ */
                const pugi::xml_node outlines_parent_node = object_node.child("outlines") ? object_node.child("outlines") : object_node;
                for (const pugi::xml_node outline_node : outlines_parent_node.children("outline"))
                {
                    RoadObjectOutline road_object_outline(outline_node.attribute("id").as_int(-1),
                                                          outline_node.attribute("fillType").as_string(""),
                                                          outline_node.attribute("laneType").as_string(""),
                                                          outline_node.attribute("outer").as_bool(true),
                                                          outline_node.attribute("closed").as_bool(true));

                    for (const pugi::xml_node corner_local_node : outline_node.children("cornerLocal"))
                    {
                        const Vec3D pt_local{corner_local_node.attribute("u").as_double(0),
                                             corner_local_node.attribute("v").as_double(0),
                                             corner_local_node.attribute("z").as_double(0)};

                        RoadObjectCorner road_object_corner_local(corner_local_node.attribute("id").as_int(-1),
                                                                  pt_local,
                                                                  corner_local_node.attribute("height").as_double(0),
                                                                  default_local_outline_type);
                        road_object_outline.outline.push_back(road_object_corner_local);
                    }

                    for (const pugi::xml_node corner_road_node : outline_node.children("cornerRoad"))
                    {
                        const Vec3D pt_road{corner_road_node.attribute("s").as_double(0),
                                            corner_road_node.attribute("t").as_double(0),
                                            corner_road_node.attribute("dz").as_double(0)};

                        RoadObjectCorner road_object_corner_road(corner_road_node.attribute("id").as_int(-1),
                                                                 pt_road,
                                                                 corner_road_node.attribute("height").as_double(0),
                                                                 RoadObjectCorner::Type::Road);
                        road_object_outline.outline.push_back(road_object_corner_road);
                    }

                    road_object.outlines.push_back(road_object_outline);
                }

                road_object.lane_validities = extract_lane_validity_records(object_node);
                road_object.additional_data = extract_additional_data(object_node);

                for (const pugi::xml_node material_node : object_node.children("material"))
                {
                    std::optional<double> friction;
                    std::optional<double> roughness;
                    if (const pugi::xml_attribute attr = material_node.attribute("friction"))
                        friction = attr.as_double();
                    if (const pugi::xml_attribute attr = material_node.attribute("roughness"))
                        roughness = attr.as_double();
                    road_object.materials.emplace_back(material_node.attribute("surface").as_string(""), friction, roughness);
                }

                if (const pugi::xml_node parking_space_node = object_node.child("parkingSpace"))
                {
                    road_object.parking_space = RoadObjectParkingSpace(parking_space_node.attribute("access").as_string(""),
                                                                       parking_space_node.attribute("restrictions").as_string(""));
                }

                for (const pugi::xml_node marking_node : object_node.child("markings").children("marking"))
                {
                    RoadObjectMarking marking(marking_node.attribute("side").as_string(""),
                                              marking_node.attribute("weight").as_string("standard"),
                                              marking_node.attribute("width").as_double(0.0),
                                              marking_node.attribute("color").as_string(""),
                                              marking_node.attribute("zOffset").as_double(0.0),
                                              marking_node.attribute("spaceLength").as_double(0.0),
                                              marking_node.attribute("lineLength").as_double(0.0),
                                              marking_node.attribute("startOffset").as_double(0.0),
                                              marking_node.attribute("stopOffset").as_double(0.0));
                    for (const pugi::xml_node corner_reference_node : marking_node.children("cornerReference"))
                        marking.corner_references.push_back(corner_reference_node.attribute("id").as_int(-1));
                    road_object.markings.push_back(marking);
                }

                for (const pugi::xml_node border_node : object_node.child("borders").children("border"))
                {
                    RoadObjectBorder border(border_node.attribute("width").as_double(0.0),
                                            border_node.attribute("type").as_string(""),
                                            border_node.attribute("outlineId").as_int(-1),
                                            border_node.attribute("useCompleteOutline").as_bool(true));
                    for (const pugi::xml_node corner_reference_node : border_node.children("cornerReference"))
                        border.corner_references.push_back(corner_reference_node.attribute("id").as_int(-1));
                    road_object.borders.push_back(border);
                }
            }

            for (const pugi::xml_node object_reference_node : road_node.child("objects").children("objectReference"))
            {
                RoadObjectReference object_reference(road_id,
                                                     object_reference_node.attribute("id").as_string(""),
                                                     object_reference_node.attribute("s").as_double(0.0),
                                                     object_reference_node.attribute("t").as_double(0.0),
                                                     object_reference_node.attribute("zOffset").as_double(0.0),
                                                     object_reference_node.attribute("validLength").as_double(0.0),
                                                     object_reference_node.attribute("orientation").as_string("none"));
                object_reference.lane_validities = extract_lane_validity_records(object_reference_node);
                road.object_references.push_back(object_reference);
            }

            for (const pugi::xml_node tunnel_node : road_node.child("objects").children("tunnel"))
            {
                const std::string tunnel_id = tunnel_node.attribute("id").as_string("");
                RoadTunnel        tunnel(road_id,
                                  tunnel_id,
                                  tunnel_node.attribute("s").as_double(0.0),
                                  tunnel_node.attribute("length").as_double(0.0),
                                  tunnel_node.attribute("name").as_string(""),
                                  tunnel_node.attribute("type").as_string(""));
                if (const pugi::xml_attribute attr = tunnel_node.attribute("lighting"))
                    tunnel.lighting = attr.as_double();
                if (const pugi::xml_attribute attr = tunnel_node.attribute("daylight"))
                    tunnel.daylight = attr.as_double();
                tunnel.lane_validities = extract_lane_validity_records(tunnel_node);
                road.id_to_tunnel.insert({tunnel_id, tunnel});
            }

            for (const pugi::xml_node bridge_node : road_node.child("objects").children("bridge"))
            {
                const std::string bridge_id = bridge_node.attribute("id").as_string("");
                RoadBridge        bridge(road_id,
                                  bridge_id,
                                  bridge_node.attribute("s").as_double(0.0),
                                  bridge_node.attribute("length").as_double(0.0),
                                  bridge_node.attribute("name").as_string(""),
                                  bridge_node.attribute("type").as_string(""));
                bridge.lane_validities = extract_lane_validity_records(bridge_node);
                road.id_to_bridge.insert({bridge_id, bridge});
            }
        }
        /* parse signals */
        if (with_road_signals)
        {
            for (const pugi::xml_node signal_node : road_node.child("signals").children("signal"))
            {
                std::string road_signal_id = signal_node.attribute("id").as_string("");
                odr::check_and_repair(
                    road.id_to_signal.find(road_signal_id) == road.id_to_signal.end(),
                    [&]() { road_signal_id = road_signal_id + std::string("_dup"); },
                    "Road #%s Signal #%s already exists, adding _dup suffix to id",
                    road_id.c_str(),
                    road_signal_id.c_str());

                RoadSignal& road_signal = road.id_to_signal
                                              .insert({road_signal_id,
                                                       RoadSignal(road_id,
                                                                  road_signal_id,
                                                                  signal_node.attribute("name").as_string(""),
                                                                  signal_node.attribute("s").as_double(0),
                                                                  signal_node.attribute("t").as_double(0),
                                                                  signal_node.attribute("dynamic").as_bool(),
                                                                  signal_node.attribute("zOffset").as_double(0),
                                                                  signal_node.attribute("value").as_double(0),
                                                                  signal_node.attribute("height").as_double(0),
                                                                  signal_node.attribute("width").as_double(0),
                                                                  signal_node.attribute("hOffset").as_double(0),
                                                                  signal_node.attribute("pitch").as_double(0),
                                                                  signal_node.attribute("roll").as_double(0),
                                                                  signal_node.attribute("orientation").as_string("none"),
                                                                  signal_node.attribute("country").as_string(""),
                                                                  signal_node.attribute("type").as_string("none"),
                                                                  signal_node.attribute("subtype").as_string("none"),
                                                                  signal_node.attribute("unit").as_string(""),
                                                                  signal_node.attribute("text").as_string("none"))})
                                              .first->second;

                for (auto [val_name, val_ptr] :
                     {std::pair{"s", &(road_signal.s0)}, std::pair{"height", &(road_signal.height)}, std::pair{"width", &(road_signal.width)}})
                {
                    odr::check_and_repair((*val_ptr) >= 0,
                                          [p = val_ptr]() { *p = 0; },
                                          "Road #%s Signal #%s: signal::%s %f < 0, set %s=0",
                                          road_id.c_str(),
                                          road_signal_id.c_str(),
                                          val_name,
                                          *val_ptr,
                                          val_name);
                }

                road_signal.lane_validities = extract_lane_validity_records(signal_node);
                road_signal.additional_data = extract_additional_data(signal_node);
                road_signal.country_revision = signal_node.attribute("countryRevision").as_string("");

                for (const pugi::xml_node dependency_node : signal_node.children("dependency"))
                {
                    road_signal.dependencies.emplace_back(dependency_node.attribute("id").as_string(""),
                                                          dependency_node.attribute("type").as_string(""));
                }

                for (const pugi::xml_node reference_node : signal_node.children("reference"))
                {
                    road_signal.references.emplace_back(reference_node.attribute("elementType").as_string(""),
                                                        reference_node.attribute("elementId").as_string(""),
                                                        reference_node.attribute("type").as_string(""));
                }

                // OpenDRIVE 1.6.1, 12.4: positionInertial and positionRoad are mutually exclusive
                const pugi::xml_node position_inertial_node = signal_node.child("positionInertial");
                const pugi::xml_node position_road_node = signal_node.child("positionRoad");
                odr::check(!(position_inertial_node && position_road_node),
                           "Road #%s Signal #%s: both positionInertial and positionRoad defined",
                           road_id.c_str(),
                           road_signal_id.c_str());
                if (position_inertial_node)
                {
                    road_signal.position_inertial = RoadSignalPositionInertial(position_inertial_node.attribute("x").as_double(0.0),
                                                                               position_inertial_node.attribute("y").as_double(0.0),
                                                                               position_inertial_node.attribute("z").as_double(0.0),
                                                                               position_inertial_node.attribute("hdg").as_double(0.0),
                                                                               position_inertial_node.attribute("pitch").as_double(0.0),
                                                                               position_inertial_node.attribute("roll").as_double(0.0));
                }
                if (position_road_node)
                {
                    road_signal.position_road = RoadSignalPositionRoad(position_road_node.attribute("roadId").as_string(""),
                                                                       position_road_node.attribute("s").as_double(0.0),
                                                                       position_road_node.attribute("t").as_double(0.0),
                                                                       position_road_node.attribute("zOffset").as_double(0.0),
                                                                       position_road_node.attribute("hOffset").as_double(0.0),
                                                                       position_road_node.attribute("pitch").as_double(0.0),
                                                                       position_road_node.attribute("roll").as_double(0.0));
                }
            }

            for (const pugi::xml_node signal_reference_node : road_node.child("signals").children("signalReference"))
            {
                RoadSignalReference signal_reference(road_id,
                                                     signal_reference_node.attribute("id").as_string(""),
                                                     signal_reference_node.attribute("s").as_double(0.0),
                                                     signal_reference_node.attribute("t").as_double(0.0),
                                                     signal_reference_node.attribute("orientation").as_string("none"));
                signal_reference.lane_validities = extract_lane_validity_records(signal_reference_node);
                road.signal_references.push_back(signal_reference);
            }
        }
    }
}

Road OpenDriveMap::get_road(const std::string& id) const
{
    return this->id_to_road.at(id);
}

Junction OpenDriveMap::get_junction(const std::string& id) const
{
    return this->id_to_junction.at(id);
}

std::vector<Road> OpenDriveMap::get_roads() const
{
    return get_map_values(this->id_to_road);
}

std::vector<Junction> OpenDriveMap::get_junctions() const
{
    return get_map_values(this->id_to_junction);
}

JunctionGroup OpenDriveMap::get_junction_group(const std::string& id) const
{
    return this->id_to_junction_group.at(id);
}

std::vector<JunctionGroup> OpenDriveMap::get_junction_groups() const
{
    return get_map_values(this->id_to_junction_group);
}

Station OpenDriveMap::get_station(const std::string& id) const
{
    return this->id_to_station.at(id);
}

std::vector<Station> OpenDriveMap::get_stations() const
{
    return get_map_values(this->id_to_station);
}

Controller OpenDriveMap::get_controller(const std::string& id) const
{
    return this->id_to_controller.at(id);
}

std::vector<Controller> OpenDriveMap::get_controllers() const
{
    return get_map_values(this->id_to_controller);
}

const LaneSection* OpenDriveMap::get_adjacent_lanesection(const std::string& road_id, const double& lanesection_s0, const bool predecessor) const
{
    const Road& road = this->id_to_road.at(road_id);
    const auto  s_lanesec_iter = road.s_to_lanesection.find(lanesection_s0);
    if (s_lanesec_iter == road.s_to_lanesection.end()) // also catches empty road
        return nullptr;

    if (predecessor)
    {
        if (s_lanesec_iter != road.s_to_lanesection.begin())
            return &(std::prev(s_lanesec_iter)->second);
    }
    else
    {
        const auto next_lanesec_iter = std::next(s_lanesec_iter);
        if (next_lanesec_iter != road.s_to_lanesection.end())
            return &(next_lanesec_iter->second);
    }

    // adjacent lanesection not in the same road
    const RoadLink& road_link = predecessor ? road.predecessor : road.successor;
    if (road_link.type == RoadLink::Type::Road && road_link.contact_point != RoadLink::ContactPoint::None)
    {
        const auto next_road_iter = this->id_to_road.find(road_link.id);
        if (next_road_iter == this->id_to_road.end())
            return nullptr;

        const Road& next_road = next_road_iter->second;
        if (next_road.s_to_lanesection.empty())
            return nullptr;

        const LaneSection& adjacent_lanesection = (road_link.contact_point == RoadLink::ContactPoint::Start)
                                                      ? next_road.s_to_lanesection.begin()->second
                                                      : next_road.s_to_lanesection.rbegin()->second;
        return &adjacent_lanesection;
    }

    return nullptr;
}

const Lane* OpenDriveMap::get_adjacent_lane(const Lane& lane, const bool predecessor) const
{
    const LaneSection* adjacent_lanesec = this->get_adjacent_lanesection(lane.key.road_id, lane.key.lanesection_s0, predecessor);
    if (!adjacent_lanesec)
        return nullptr;

    const std::optional<int> adjacent_lane_id = predecessor ? lane.predecessor : lane.successor;
    if (adjacent_lane_id)
    {
        const auto adjacent_lane_iter = adjacent_lanesec->id_to_lane.find(adjacent_lane_id.value());
        if (adjacent_lane_iter != adjacent_lanesec->id_to_lane.end())
            return &(adjacent_lane_iter->second);
    }

    return nullptr;
}

std::vector<const Lane*> OpenDriveMap::get_adjacent_lanes(const Lane& lane, const bool predecessor) const
{
    std::vector<const Lane*> adjacent_lanes;

    const LaneSection* adjacent_lanesec = this->get_adjacent_lanesection(lane.key.road_id, lane.key.lanesection_s0, predecessor);
    if (!adjacent_lanesec)
        return adjacent_lanes;

    for (const int adjacent_lane_id : predecessor ? lane.predecessors : lane.successors)
    {
        const auto adjacent_lane_iter = adjacent_lanesec->id_to_lane.find(adjacent_lane_id);
        if (adjacent_lane_iter != adjacent_lanesec->id_to_lane.end())
            adjacent_lanes.push_back(&(adjacent_lane_iter->second));
    }

    return adjacent_lanes;
}

RoadNetworkMesh OpenDriveMap::get_road_network_mesh(const double eps) const
{
    RoadNetworkMesh  out_mesh;
    LanesMesh&       lanes_mesh = out_mesh.lanes_mesh;
    RoadmarksMesh&   roadmarks_mesh = out_mesh.roadmarks_mesh;
    RoadObjectsMesh& road_objects_mesh = out_mesh.road_objects_mesh;
    RoadSignalsMesh& road_signals_mesh = out_mesh.road_signals_mesh;

    for (const auto& id_road : this->id_to_road)
    {
        const Road& road = id_road.second;
        lanes_mesh.road_start_indices[lanes_mesh.vertices.size()] = road.id;
        roadmarks_mesh.road_start_indices[roadmarks_mesh.vertices.size()] = road.id;
        road_objects_mesh.road_start_indices[road_objects_mesh.vertices.size()] = road.id;

        for (const auto& s_lanesec : road.s_to_lanesection)
        {
            const LaneSection& lanesec = s_lanesec.second;
            lanes_mesh.lanesec_start_indices[lanes_mesh.vertices.size()] = lanesec.s0;
            roadmarks_mesh.lanesec_start_indices[roadmarks_mesh.vertices.size()] = lanesec.s0;
            for (const auto& id_lane : lanesec.id_to_lane)
            {
                const Lane&       lane = id_lane.second;
                const std::size_t lanes_idx_offset = lanes_mesh.vertices.size();
                lanes_mesh.lane_start_indices[lanes_idx_offset] = lane.id;
                lanes_mesh.add_mesh(road.get_lane_mesh(lane, eps));

                std::size_t roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                roadmarks_mesh.lane_start_indices[roadmarks_idx_offset] = lane.id;
                const std::vector<RoadMark> roadmarks = lane.get_roadmarks(lanesec.s0, road.get_lanesection_end(lanesec));
                for (const RoadMark& roadmark : roadmarks)
                {
                    roadmarks_idx_offset = roadmarks_mesh.vertices.size();
                    roadmarks_mesh.roadmark_type_start_indices[roadmarks_idx_offset] = roadmark.type;
                    roadmarks_mesh.roadmark_color_start_indices[roadmarks_idx_offset] = roadmark.color;
                    roadmarks_mesh.add_mesh(road.get_roadmark_mesh(lane, roadmark, eps));
                }
            }
        }

        for (const auto& id_road_object : road.id_to_object)
        {
            const RoadObject& road_object = id_road_object.second;
            const std::size_t road_objs_idx_offset = road_objects_mesh.vertices.size();
            road_objects_mesh.road_object_start_indices[road_objs_idx_offset] = road_object.id;
            road_objects_mesh.add_mesh(road.get_road_object_mesh(road_object, eps));
        }

        for (const auto& id_signal : road.id_to_signal)
        {
            const RoadSignal& road_signal = id_signal.second;
            const std::size_t signals_idx_offset = road_signals_mesh.vertices.size();
            road_signals_mesh.road_signal_start_indices[signals_idx_offset] = road_signal.id;
            road_signals_mesh.add_mesh(road.get_road_signal_mesh(road_signal));
        }
    }

    return out_mesh;
}

RoutingGraph OpenDriveMap::get_routing_graph() const
{
    RoutingGraph routing_graph;

    // Parse Roads
    for (const auto& [_, road] : id_to_road)
    {
        for (const auto& [_, lanesection] : road.s_to_lanesection)
        {
            for (const auto& [_, lane] : lanesection.id_to_lane)
            {
                const bool lane_follows_road_direction = lane.key.lane_id < 0;

                for (const Lane* predecessor_lane : this->get_adjacent_lanes(lane, lane_follows_road_direction))
                {
                    const Road&  predecessor_road = this->id_to_road.at(predecessor_lane->key.road_id);
                    const double lane_length = predecessor_road.get_lanesection_length(predecessor_lane->key.lanesection_s0);
                    routing_graph.add_edge(RoutingGraphEdge(predecessor_lane->key, lane.key, lane_length));
                }

                for (const Lane* successor_lane : this->get_adjacent_lanes(lane, !lane_follows_road_direction))
                {
                    const double lane_length = road.get_lanesection_length(lane.key.lanesection_s0);
                    routing_graph.add_edge(RoutingGraphEdge(lane.key, successor_lane->key, lane_length));
                }
            }
        }
    }

    // Parse Junctions
    for (const auto& [_, junction] : id_to_junction)
    {
        for (const auto& [_, conn] : junction.id_to_connection)
        {
            auto incoming_road_iter = id_to_road.find(conn.incoming_road);
            auto connecting_road_iter = id_to_road.find(conn.connecting_road);
            if (incoming_road_iter == id_to_road.end() || connecting_road_iter == id_to_road.end())
                continue;

            const Road& incoming_road = incoming_road_iter->second;
            const Road& connecting_road = connecting_road_iter->second;

            const LaneSection& incoming_lanesec =
                (incoming_road.successor.type == RoadLink::Type::Junction && incoming_road.successor.id == junction.id)
                    ? incoming_road.s_to_lanesection.rbegin()->second
                    : incoming_road.s_to_lanesection.begin()->second;

            const LaneSection& connecting_lanesec = (conn.contact_point == JunctionConnection::ContactPoint::Start)
                                                        ? connecting_road.s_to_lanesection.begin()->second
                                                        : connecting_road.s_to_lanesection.rbegin()->second;

            for (const JunctionLaneLink& lane_link : conn.lane_links)
            {
                if (lane_link.from == 0 || lane_link.to == 0)
                    continue;

                auto from_lane_iter = incoming_lanesec.id_to_lane.find(lane_link.from);
                auto to_lane_iter = connecting_lanesec.id_to_lane.find(lane_link.to);

                if (from_lane_iter == incoming_lanesec.id_to_lane.end() || to_lane_iter == connecting_lanesec.id_to_lane.end())
                    continue;

                const LaneKey from(incoming_road.id, incoming_lanesec.s0, from_lane_iter->second.id);
                const LaneKey to(connecting_road.id, connecting_lanesec.s0, to_lane_iter->second.id);
                const double  lane_length = incoming_road.get_lanesection_length(incoming_lanesec);

                routing_graph.add_edge(RoutingGraphEdge(from, to, lane_length));
            }
        }
    }

    return routing_graph;
}

} // namespace odr
