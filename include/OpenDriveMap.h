#pragma once
#include "Header.h"
#include "Junction.h"
#include "Road.h"
#include "RoadNetworkMesh.h"
#include "RoutingGraph.h"

#include "pugixml.hpp"

#include <map>
#include <string>
#include <vector>

namespace odr
{

class OpenDriveMap
{
public:
    OpenDriveMap(const std::string& xodr_file,
                 const bool         center_map = false,
                 const bool         with_road_objects = true,
                 const bool         with_lateral_profile = true,
                 const bool         with_lane_height = true,
                 const bool         abs_z_for_for_local_road_obj_outline = false,
                 const bool         fix_spiral_edge_cases = true,
                 const bool         with_road_signals = true);

    Road                       get_road(const std::string& id) const;
    std::vector<Road>          get_roads() const;
    Junction                   get_junction(const std::string& id) const;
    std::vector<Junction>      get_junctions() const;
    JunctionGroup              get_junction_group(const std::string& id) const;
    std::vector<JunctionGroup> get_junction_groups() const;
    Controller                 get_controller(const std::string& id) const;
    std::vector<Controller>    get_controllers() const;
    Station                    get_station(const std::string& id) const;
    std::vector<Station>       get_stations() const;

    const LaneSection*       get_adjacent_lanesection(const std::string& road_id, const double& lanesection_s0, const bool predecessor) const;
    const Lane*              get_adjacent_lane(const Lane& lane, const bool predecessor) const;
    std::vector<const Lane*> get_adjacent_lanes(const Lane& lane, const bool predecessor) const;

    // OpenDRIVE 1.6.1, 12.4: signal box at its physical position, resolving positionRoad on other roads
    Mesh3D          get_road_signal_mesh(const RoadSignal& road_signal) const;
    RoadNetworkMesh get_road_network_mesh(const double eps) const;
    RoutingGraph    get_routing_graph() const;

    Header            header;
    AdditionalData    additional_data; // <userData>, <include> and <dataQuality> directly within <OpenDRIVE>
    std::string       proj4 = "";
    double            x_offs = 0;
    double            y_offs = 0;
    const std::string xodr_file = "";

    std::map<std::string, Road>          id_to_road;
    std::map<std::string, Junction>      id_to_junction;
    std::map<std::string, JunctionGroup> id_to_junction_group;
    std::map<std::string, Controller>    id_to_controller;
    std::map<std::string, Station>       id_to_station; // OpenDRIVE 1.6.1, 13.3
};

} // namespace odr