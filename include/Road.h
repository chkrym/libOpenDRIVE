#pragma once
#include "Geometries/CubicSpline.h"
#include "LaneSection.h"
#include "Math.hpp"
#include "Mesh.h"
#include "Railroad.h"
#include "RefLine.h"
#include "RoadObject.h"
#include "RoadSignal.h"
#include "SurfaceCRG.h"

#include <map>
#include <optional>
#include <set>
#include <stdint.h>
#include <string>
#include <vector>

namespace odr
{

struct Lane;
struct RoadMark;

struct Crossfall : public CubicProfile
{
    enum class Side
    {
        Both,
        Left,
        Right
    };

    Crossfall() = default;

    double get_crossfall(const double s, const bool on_left_side) const;

    std::map<double, Side> sides;
};

struct RoadLink
{
    enum class ContactPoint
    {
        None,
        Start,
        End
    };

    enum class Type
    {
        None,
        Road,
        Junction
    };

    RoadLink() = default;
    RoadLink(std::string id, Type type, ContactPoint contact_point);

    std::string  id = "";
    Type         type = Type::None;
    ContactPoint contact_point = ContactPoint::None;

    // OpenDRIVE 1.6.1, 8.2 Road linkage: alternative to @contactPoint for virtual junctions, the road is linked within the
    // linked road, not at its start or end
    std::optional<double>      element_s;
    std::optional<std::string> element_dir; // "+" or "-", direction on the linked road from which the road is entered
};

struct RoadNeighbor
{
    RoadNeighbor(std::string id, std::string side, std::string direction);

    std::string id = "";
    std::string side = "";
    std::string direction = "";
};

struct SpeedRecord
{
    SpeedRecord(std::string max, std::string unit);

    std::string max = "";
    std::string unit = "";
};

class Road
{
public:
    Road(std::string id, double length, std::string junction, std::string name, bool left_hand_traffic = false);

    std::vector<LaneSection> get_lanesections() const;
    std::vector<RoadObject>  get_road_objects() const;
    std::vector<RoadSignal>  get_road_signals() const;

    double      get_lanesection_s0(const double s) const;
    LaneSection get_lanesection(const double s) const;

    double get_lanesection_end(const LaneSection& lanesection) const;
    double get_lanesection_end(const double lanesection_s0) const;
    double get_lanesection_length(const LaneSection& lanesection) const;
    double get_lanesection_length(const double lanesection_s0) const;

    Vec3D get_xyz(const double s, const double t, const double h, Vec3D* e_s = nullptr, Vec3D* e_t = nullptr, Vec3D* e_h = nullptr) const;
    Vec3D get_surface_pt(double s, const double t, Vec3D* vn = nullptr) const;

    Line3D get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer = true) const;
    Line3D get_lane_border_line(const Lane& lane, const double eps, const bool outer = true) const;

    Mesh3D get_lane_mesh(
        const Lane& lane, const double s_start, const double s_end, const double eps, std::vector<uint32_t>* outline_indices = nullptr) const;
    Mesh3D get_lane_mesh(const Lane& lane, const double eps, std::vector<uint32_t>* outline_indices = nullptr) const;

    Mesh3D get_roadmark_mesh(const Lane& lane, const RoadMark& roadmark, const double eps) const;
    // OpenDRIVE 1.6.1, 12.4: at positionInertial, or positionRoad on this road, if present; logical position otherwise
    Mesh3D get_road_signal_mesh(const RoadSignal& road_signal) const;
    // signal box at the given road coordinates of this road
    Mesh3D get_road_signal_mesh(const RoadSignal& road_signal,
                                double            s,
                                const double      t,
                                const double      z_offset,
                                const double      h_offset,
                                const double      pitch,
                                const double      roll) const;
    Mesh3D get_road_object_mesh(const RoadObject& road_object, const double eps) const;

    std::set<double>
    approximate_lane_border_linear(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer = true) const;
    std::set<double> approximate_lane_border_linear(const Lane& lane, const double eps, const bool outer = true) const;

    double           get_lateral_shape(const double s, const double t) const;
    double           get_lateral_shape_derivative(const double s, const double t) const;
    std::set<double> approximate_lateral_shape_linear(const double s, const double t_start, const double t_end, const double eps) const;

    double      length = 0;
    std::string id = "";
    std::string junction = "";
    std::string name = "";
    bool        left_hand_traffic = false;

    RoadLink                  predecessor;
    RoadLink                  successor;
    std::vector<RoadNeighbor> neighbors;

    CubicProfile lane_offset;
    CubicProfile superelevation;
    Crossfall    crossfall;
    RefLine      ref_line;

    std::map<double /*s0*/, CubicProfile /*polys over t*/> s_to_lateral_shape;

    std::map<double, LaneSection>     s_to_lanesection;
    std::map<double, std::string>     s_to_type;
    std::map<double, std::string>     s_to_country; // OpenDRIVE 1.6.1, 8.3 <type @country>, only for records that set it
    std::map<double, SpeedRecord>     s_to_speed;
    std::map<std::string, RoadObject> id_to_object;

    std::vector<RoadObjectReference>  object_references; // OpenDRIVE 1.6.1, 11.8, ids may repeat across roads
    std::map<std::string, RoadTunnel> id_to_tunnel;
    std::map<std::string, RoadBridge> id_to_bridge;
    std::map<std::string, RoadSignal> id_to_signal;
    std::vector<RoadSignalReference>  signal_references; // OpenDRIVE 1.6.1, 12.5, ids may repeat across roads

    std::map<std::string, RailroadSwitch> id_to_switch; // OpenDRIVE 1.6.1, 13.2 <railroad><switch>

    std::vector<SurfaceCRG> surface_crgs; // OpenDRIVE 1.6.1, 8.5 <surface><CRG>
    AdditionalData          additional_data;
};

} // namespace odr
