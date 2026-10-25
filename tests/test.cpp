#include "OpenDriveMap.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <string>
#include <utility>

struct OpenDriveFixture
{
    OpenDriveFixture() : odr_map("test.xodr")
    {
        REQUIRE(!odr_map.get_roads().empty());
    }

    odr::OpenDriveMap odr_map;
};

TEST_CASE_METHOD(OpenDriveFixture, "Basic OpenDriveMap check", "[xodr]")
{
    // basic routing test
    auto graph = odr_map.get_routing_graph();
    auto path = graph.shortest_path(odr::LaneKey("43", 0.0, 1), odr::LaneKey("41", 0.0, 1));
    REQUIRE(path.size() == 15);

    // road sanity
    for (const odr::Road& road : odr_map.get_roads())
    {
        INFO("road: " << road.id << ", length: " << road.length);
        REQUIRE(road.length >= 0.0);
        REQUIRE(!road.s_to_lanesection.empty());
        for (const odr::LaneSection& ls : road.get_lanesections())
        {
            const double s_start = ls.s0;
            const double s_end = road.get_lanesection_end(ls);
            REQUIRE(s_start >= 0.0);
            REQUIRE(s_end > s_start);
            for (const odr::Lane& lane : ls.get_lanes())
            {
                std::vector<odr::RoadMark> roadmarks = lane.get_roadmarks(s_start, s_end);
                (void)roadmarks; // silence unused var if not checked
            }
        }
    }
}

TEST_CASE_METHOD(OpenDriveFixture, "Routing check", "[xodr]")
{
    const std::vector<odr::RoutingPath> expected_paths = {
        {{"37", 0, 1}, {"215", 0, -1}, {"47", 0, -1}},
        {{"24", 0, -1}, {"447", 0, -1}, {"20", 0, -1}, {"765", 0, -1}, {"21", 0, -1}, {"39", 0, -1}},
        {{"17", 0, -1},
         {"18", 0, -1},
         {"747", 0, -1},
         {"747", 1.7623724829973639, -1},
         {"41", 0, 1},
         {"388", 0, -1},
         {"35", 0, 1},
         {"480", 0, -1},
         {"480", 20.515904612542592, -1},
         {"480", 20.858538341635004, -1},
         {"53", 0, 1},
         {"817", 0, -1},
         {"63", 0, -4},
         {"64", 0, -4},
         {"65", 0, -4}}};

    const odr::RoutingGraph graph = odr_map.get_routing_graph();
    for (const odr::RoutingPath& expected_path : expected_paths)
    {
        const odr::RoutingPath path = graph.shortest_path(expected_path.front(), expected_path.back());
        REQUIRE(path == expected_path);
    }
}

TEST_CASE("Lane border definitions", "[xodr][lane_border]")
{
    odr::OpenDriveMap       odr_map("lane_border.xodr");
    const odr::Road&        road = odr_map.get_road("1");
    const odr::LaneSection& ls = road.get_lanesection(0.0);

    const odr::Lane& lane_1 = ls.get_lane(1);
    const odr::Lane& lane_2 = ls.get_lane(2);
    const odr::Lane& lane_m1 = ls.get_lane(-1);

    REQUIRE(lane_1.outer_border.evaluate(10.0) == Catch::Approx(3.5));
    REQUIRE(lane_2.outer_border.evaluate(10.0) == Catch::Approx(7.0));
    REQUIRE(lane_2.outer_border.evaluate(75.0) == Catch::Approx(7.5));

    // widths are derived from borders
    REQUIRE(lane_1.lane_width.evaluate(10.0) == Catch::Approx(3.5));
    REQUIRE(lane_2.lane_width.evaluate(10.0) == Catch::Approx(3.5));
    REQUIRE(lane_2.lane_width.evaluate(75.0) == Catch::Approx(4.0));

    // <width> takes precedence over <border>
    REQUIRE(lane_m1.outer_border.evaluate(10.0) == Catch::Approx(-3.0));

    REQUIRE(ls.get_lane_id(10.0, 5.0) == 2);
    REQUIRE(ls.get_lane_id(10.0, 1.0) == 1);

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

TEST_CASE("Single side lane sections", "[xodr][single_side]")
{
    odr::OpenDriveMap odr_map("single_side.xodr");
    const odr::Road   road = odr_map.get_road("1");

    const odr::LaneSection ls_single = road.get_lanesection(30.0);
    REQUIRE(ls_single.id_to_lane.size() == 4);

    // lane #-1 continues from the lane section at s=0
    const odr::Lane lane_m1 = ls_single.get_lane(-1);
    REQUIRE(lane_m1.key.lanesection_s0 == 30.0);
    REQUIRE(lane_m1.outer_border.evaluate(40.0) == Catch::Approx(-4.0));
    REQUIRE(ls_single.get_lane(2).outer_border.evaluate(40.0) == Catch::Approx(7.0));

    // lane #-1 is connected through all three lane sections
    const odr::RoutingGraph routing_graph = odr_map.get_routing_graph();
    const auto              path = routing_graph.shortest_path(odr::LaneKey("1", 0.0, -1), odr::LaneKey("1", 60.0, -1));
    REQUIRE(path.size() == 3);

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

TEST_CASE("Multiple lane predecessors and successors", "[xodr][lane_link]")
{
    const odr::OpenDriveMap odr_map("lane_multi_link.xodr");

    const odr::Lane split = odr_map.get_road("1").get_lanesection(0.0).get_lane(-1);
    REQUIRE(split.successors == std::vector<int>{-1, -2});
    REQUIRE(split.successor == -1); // first link
    REQUIRE(split.predecessors.empty());
    REQUIRE(!split.predecessor.has_value());

    const odr::Lane merge = odr_map.get_road("2").get_lanesection(0.0).get_lane(-1);
    REQUIRE(merge.predecessors == std::vector<int>{-1, -2});

    const std::vector<const odr::Lane*> successors = odr_map.get_adjacent_lanes(split, false);
    REQUIRE(successors.size() == 2);
    REQUIRE(successors.at(1)->key == odr::LaneKey("1", 25.0, -2));
    REQUIRE(odr_map.get_adjacent_lanes(merge, true).size() == 2);

    const odr::RoutingGraph routing_graph = odr_map.get_routing_graph();

    // the split is in the routing graph in both directions of the reference line
    std::vector<odr::LaneKey> lane_successors = routing_graph.get_lane_successors(odr::LaneKey("1", 0.0, -1));
    REQUIRE(lane_successors.size() == 2);
    std::vector<odr::LaneKey> lane_predecessors = routing_graph.get_lane_predecessors(odr::LaneKey("1", 0.0, 1));
    REQUIRE(lane_predecessors.size() == 2);

    // both lanes after the split merge into road 2
    REQUIRE(routing_graph.get_lane_predecessors(odr::LaneKey("2", 0.0, -1)).size() == 2);
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("1", 25.0, -2), odr::LaneKey("2", 0.0, -1)).size() == 2);
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("1", 0.0, -1), odr::LaneKey("2", 0.0, -1)).size() == 3);
}

TEST_CASE("Left-hand traffic routing", "[xodr][routing][lht]")
{
    const odr::OpenDriveMap odr_map("traffic_rule.xodr");
    REQUIRE(!odr_map.get_road("1").left_hand_traffic);
    REQUIRE(odr_map.get_road("2").left_hand_traffic);

    const odr::RoutingGraph routing_graph = odr_map.get_routing_graph();

    // RHT: right lanes drive along the reference line, left lanes against it
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("1", 0.0, -1), odr::LaneKey("1", 25.0, -1)).size() == 2);
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("1", 25.0, 1), odr::LaneKey("1", 0.0, 1)).size() == 2);
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("1", 0.0, 1), odr::LaneKey("1", 25.0, 1)).empty());

    // LHT: left lanes drive along the reference line, right lanes against it
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("2", 0.0, 1), odr::LaneKey("2", 25.0, 1)).size() == 2);
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("2", 25.0, -1), odr::LaneKey("2", 0.0, -1)).size() == 2);
    REQUIRE(routing_graph.shortest_path(odr::LaneKey("2", 0.0, -1), odr::LaneKey("2", 25.0, -1)).empty());
    REQUIRE(routing_graph.get_lane_successors(odr::LaneKey("2", 0.0, 1)) == std::vector<odr::LaneKey>{odr::LaneKey("2", 25.0, 1)});
}

TEST_CASE("Poly3 geometry", "[xodr][poly3]")
{
    odr::OpenDriveMap odr_map("poly3.xodr", false);

    // straight line with a != 0: the curve still starts at (@x,@y) with heading @hdg (90 degrees)
    const odr::Road  straight = odr_map.get_road("1");
    const odr::Vec3D start = straight.get_xyz(0.0, 0.0, 0.0);
    REQUIRE(start[0] == Catch::Approx(10.0));
    REQUIRE(start[1] == Catch::Approx(0.0).margin(1e-9));
    const odr::Vec3D end = straight.get_xyz(50.0, 0.0, 0.0);
    REQUIRE(end[0] == Catch::Approx(10.0));
    REQUIRE(end[1] == Catch::Approx(50.0));

    // curved with a = b = 0: end point lies on v(u) and the reference line length matches @length
    const odr::Road  curved = odr_map.get_road("2");
    const odr::Vec3D curved_end = curved.get_xyz(35.0, 0.0, 0.0);
    const double     u = curved_end[0];
    REQUIRE(curved_end[1] == Catch::Approx(0.002 * u * u - 0.00003 * u * u * u));

    double     arc_length = 0.0;
    odr::Vec3D prev = curved.get_xyz(0.0, 0.0, 0.0);
    for (int i = 1; i <= 3500; i++)
    {
        const odr::Vec3D pt = curved.get_xyz(i * 0.01, 0.0, 0.0);
        arc_length += std::hypot(pt[0] - prev[0], pt[1] - prev[1]);
        prev = pt;
    }
    REQUIRE(arc_length == Catch::Approx(35.0).epsilon(1e-4));

    // rotated u/v system with a, b != 0: starts at (@x,@y) with heading @hdg, a straight v(u) stays on the @hdg direction
    const odr::Road  rotated = odr_map.get_road("3");
    const odr::Vec3D rotated_start = rotated.get_xyz(0.0, 0.0, 0.0);
    REQUIRE(rotated_start[0] == Catch::Approx(5.0));
    REQUIRE(rotated_start[1] == Catch::Approx(5.0));
    const odr::Vec3D rotated_end = rotated.get_xyz(20.0, 0.0, 0.0);
    REQUIRE(rotated_end[0] == Catch::Approx(5.0 + 20.0 * std::cos(0.3)));
    REQUIRE(rotated_end[1] == Catch::Approx(5.0 + 20.0 * std::sin(0.3)));

    // curved with b != 0: start heading equals @hdg
    const odr::Road  rotated_curved = odr_map.get_road("4");
    const odr::Vec3D grad = rotated_curved.ref_line.derivative(0.0);
    REQUIRE(std::atan2(grad[1], grad[0]) == Catch::Approx(0.3));
    REQUIRE(rotated_curved.get_xyz(0.0, 0.0, 0.0)[0] == Catch::Approx(5.0));

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

TEST_CASE("Lateral profile shape", "[xodr][lateral_shape]")
{
    odr::OpenDriveMap odr_map("lateral_shape.xodr", false);
    const odr::Road   road = odr_map.get_road("1");

    REQUIRE(road.s_to_lateral_shape.size() == 2);

    // s=0: h(t=-4)=0, h(t=-2)=0.25, h(t=0)=1, h(t=2)=0.75
    REQUIRE(road.get_lateral_shape(0.0, -4.0) == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(road.get_lateral_shape(0.0, -2.0) == Catch::Approx(0.25));
    REQUIRE(road.get_lateral_shape(0.0, 0.0) == Catch::Approx(1.0));
    REQUIRE(road.get_lateral_shape(0.0, 2.0) == Catch::Approx(0.75));

    // linear interpolation towards the flat profile at s=100
    REQUIRE(road.get_lateral_shape(50.0, 0.0) == Catch::Approx(0.5));
    REQUIRE(road.get_lateral_shape(100.0, 0.0) == Catch::Approx(0.0).margin(1e-9));

    REQUIRE(road.get_surface_pt(50.0, 0.0)[2] == Catch::Approx(0.5));

    // dh/dt at s=0: t=-2 -> 0.25, t=2 -> -0.25; the surface normal is tilted accordingly
    REQUIRE(road.get_lateral_shape_derivative(0.0, -2.0) == Catch::Approx(0.25));
    REQUIRE(road.get_lateral_shape_derivative(0.0, 2.0) == Catch::Approx(-0.25));
    REQUIRE(road.get_lateral_shape_derivative(50.0, 2.0) == Catch::Approx(-0.125));

    odr::Vec3D vn{0, 0, 0};
    road.get_surface_pt(0.0, 2.0, &vn);
    const double inv_norm = 1.0 / std::sqrt(1.0 + 0.25 * 0.25);
    REQUIRE(vn[0] == Catch::Approx(0.0).margin(1e-9));
    REQUIRE(vn[1] == Catch::Approx(0.25 * inv_norm));
    REQUIRE(vn[2] == Catch::Approx(inv_norm));

    road.get_surface_pt(0.0, -2.0, &vn); // rising towards the crown
    REQUIRE(vn[1] == Catch::Approx(-0.25 * inv_norm));

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

static double get_mesh_xy_area(const odr::Mesh3D& mesh, std::size_t* num_downward_faces)
{
    double area = 0;
    *num_downward_faces = 0;
    for (std::size_t idx = 0; idx + 2 < mesh.indices.size(); idx += 3)
    {
        const odr::Vec3D& a = mesh.vertices.at(mesh.indices.at(idx));
        const odr::Vec3D& b = mesh.vertices.at(mesh.indices.at(idx + 1));
        const odr::Vec3D& c = mesh.vertices.at(mesh.indices.at(idx + 2));
        const double      cross_z = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
        if (cross_z < 0)
            (*num_downward_faces)++;
        area += 0.5 * std::abs(cross_z);
    }
    return area;
}

TEST_CASE("Lane mesh is subdivided along t for lateral shapes", "[xodr][lateral_shape][mesh]")
{
    odr::OpenDriveMap      odr_map("lateral_shape.xodr", false);
    const odr::Road        road = odr_map.get_road("1");
    const odr::LaneSection lanesection = road.get_lanesection(0.0);
    const double           eps = 0.01;

    REQUIRE(road.approximate_lateral_shape_linear(-1.0, -4.0, 0.0, eps).size() == 2); // no shape before s=0
    REQUIRE(road.approximate_lateral_shape_linear(0.0, -4.0, 4.0, eps).size() > 3);

    for (const int lane_id : {-1, 1})
    {
        const odr::Lane       lane = lanesection.get_lane(lane_id);
        std::vector<uint32_t> outline_indices;
        const odr::Mesh3D     mesh = road.get_lane_mesh(lane, eps, &outline_indices);

        // more than the two lane border points per cross section at s=0, where the shape is curved
        std::size_t num_s0_pts = 0;
        for (const odr::Vec2D& st : mesh.st_coordinates)
            if (st[0] == 0.0)
                num_s0_pts++;
        REQUIRE(num_s0_pts > 2);

        // vertices lie on the surface, midpoints between neighbouring points of a cross section are within eps of the shape
        for (std::size_t idx = 0; idx < mesh.vertices.size(); idx++)
        {
            const odr::Vec2D& st = mesh.st_coordinates.at(idx);
            REQUIRE(mesh.vertices.at(idx)[2] == Catch::Approx(road.get_lateral_shape(st[0], st[1])).margin(1e-9));
            if (idx + 1 < mesh.vertices.size() && mesh.st_coordinates.at(idx + 1)[0] == st[0])
            {
                const double t_mid = 0.5 * (st[1] + mesh.st_coordinates.at(idx + 1)[1]);
                const double z_mid = 0.5 * (mesh.vertices.at(idx)[2] + mesh.vertices.at(idx + 1)[2]);
                REQUIRE(std::abs(z_mid - road.get_lateral_shape(st[0], t_mid)) <= eps + 1e-9);
            }
        }

        // the triangles cover the lane exactly once and all face up
        std::size_t num_downward_faces = 0;
        REQUIRE(get_mesh_xy_area(mesh, &num_downward_faces) == Catch::Approx(400.0));
        REQUIRE(num_downward_faces == 0);

        REQUIRE(!outline_indices.empty());
        for (const uint32_t idx : outline_indices)
            REQUIRE(idx < mesh.vertices.size());
    }
}

TEST_CASE("Lane outlines of the road network mesh follow subdivided lane meshes", "[xodr][lateral_shape][mesh]")
{
    for (const std::string xodr_file : {"lateral_shape.xodr", "lane_border.xodr"})
    {
        odr::OpenDriveMap         odr_map(xodr_file, false);
        const odr::LanesMesh      lanes_mesh = odr_map.get_road_network_mesh(0.01).lanes_mesh;
        const std::vector<size_t> outline_indices = lanes_mesh.get_lane_outline_indices();
        REQUIRE(outline_indices.size() % 2 == 0);

        // the outline consists exactly of the boundary edges (used by one triangle) of each lane mesh
        std::map<std::pair<size_t, size_t>, int> edge_counts;
        for (std::size_t idx = 0; idx + 2 < lanes_mesh.indices.size(); idx += 3)
        {
            for (std::size_t corner = 0; corner < 3; corner++)
            {
                const size_t a = lanes_mesh.indices.at(idx + corner);
                const size_t b = lanes_mesh.indices.at(idx + (corner + 1) % 3);
                edge_counts[{std::min(a, b), std::max(a, b)}]++;
            }
        }
        std::set<std::pair<size_t, size_t>> boundary_edges;
        for (const auto& edge_count : edge_counts)
            if (edge_count.second == 1)
                boundary_edges.insert(edge_count.first);

        std::set<std::pair<size_t, size_t>> outline_edges;
        for (std::size_t idx = 0; idx + 1 < outline_indices.size(); idx += 2)
        {
            const size_t a = outline_indices.at(idx);
            const size_t b = outline_indices.at(idx + 1);
            REQUIRE(lanes_mesh.get_idx_interval_lane(a) == lanes_mesh.get_idx_interval_lane(b));
            outline_edges.insert({std::min(a, b), std::max(a, b)});
        }
        REQUIRE(outline_edges == boundary_edges);
    }
}

TEST_CASE("Lane mesh without lateral shape stays a triangle strip", "[xodr][mesh]")
{
    odr::OpenDriveMap odr_map("lane_border.xodr", false);
    for (const odr::Road& road : odr_map.get_roads())
    {
        for (const odr::LaneSection& lanesection : road.get_lanesections())
        {
            for (const odr::Lane& lane : lanesection.get_lanes())
            {
                std::vector<uint32_t> outline_indices;
                const odr::Mesh3D     mesh = road.get_lane_mesh(lane, 0.1, &outline_indices);
                REQUIRE(mesh.vertices.size() % 2 == 0);
                REQUIRE(mesh.indices.size() == 3 * (mesh.vertices.size() - 2));
                REQUIRE(outline_indices == odr::get_triangle_strip_outline_indices<uint32_t>(mesh.vertices.size()));
                std::size_t num_downward_faces = 0;
                get_mesh_xy_area(mesh, &num_downward_faces);
                REQUIRE(num_downward_faces == 0);
            }
        }
    }
}

TEST_CASE("Level lanes exclude superelevation and lateral shape", "[xodr][lateral_shape][level]")
{
    odr::OpenDriveMap odr_map("level_shape.xodr", false);
    const odr::Road   road = odr_map.get_road("1");

    const double superelev = 0.05;
    // lane -1 is not level: superelevation and shape apply
    REQUIRE(road.get_surface_pt(10.0, -3.0)[2] == Catch::Approx(-3.0 * std::sin(superelev) + 0.0625 * std::cos(superelev)));

    // OpenDRIVE 1.6.1, 9.5.8: the level lanes -2 and -3 stay on the height of the inner border of lane -2 (t=-4, shape h=0),
    // neither the shape (h(t=-7)=0.5625) nor superelevation is applied to them
    const double z_level = -4.0 * std::sin(superelev);
    for (const double t : {-4.5, -5.5, -6.5, -7.0, -7.5})
    {
        odr::Vec3D       vn{0, 0, 0};
        const odr::Vec3D pt = road.get_surface_pt(10.0, t, &vn);
        REQUIRE(pt[2] == Catch::Approx(z_level));
        REQUIRE(vn[0] == Catch::Approx(0.0).margin(1e-9));
        REQUIRE(vn[1] == Catch::Approx(0.0).margin(1e-9));
        REQUIRE(vn[2] == Catch::Approx(1.0));
    }

    const odr::Mesh3D lane_mesh = road.get_lane_mesh(road.get_lanesection(0.0).get_lane(-3), 0.1);
    REQUIRE(!lane_mesh.vertices.empty());
    for (const odr::Vec3D& vertex : lane_mesh.vertices)
        REQUIRE(vertex[2] == Catch::Approx(z_level));
}

TEST_CASE("Header attributes and offset", "[xodr][header]")
{
    const odr::OpenDriveMap odr_map("header.xodr");
    const odr::Header&      header = odr_map.header;

    REQUIRE(header.rev_major == 1);
    REQUIRE(header.rev_minor == 6);
    REQUIRE(header.name == "header_test");
    REQUIRE(header.version == "1.00");
    REQUIRE(header.date == "2026-10-23T12:00:00");
    REQUIRE(header.vendor == "libOpenDRIVE");
    REQUIRE(header.north == 100.0);
    REQUIRE(header.south == -10.0);
    REQUIRE(header.east == 200.0);
    REQUIRE(header.west == -20.0);

    REQUIRE(header.offset.has_value());
    REQUIRE(header.offset->x == 500000.0);
    REQUIRE(header.offset->y == 5400000.0);
    REQUIRE(header.offset->z == 100.0);
    REQUIRE(header.offset->hdg == 0.5);

    REQUIRE(odr_map.proj4.find("+proj=utm") != std::string::npos);

    // optional values stay empty when not given
    const odr::OpenDriveMap other_map("lane_border.xodr");
    REQUIRE(!other_map.header.offset.has_value());
    REQUIRE(!other_map.header.north.has_value());
}

TEST_CASE("Lane material, speed, access and rule", "[xodr][lane_attributes]")
{
    const odr::OpenDriveMap odr_map("lane_attributes.xodr");
    const odr::Road         road = odr_map.get_road("1");

    // road type speed without unit defaults to m/s as well (OpenDRIVE 1.6.1, 2.3.2)
    REQUIRE(road.s_to_speed.at(0.0).unit == "m/s");

    // road type country code (OpenDRIVE 1.6.1, 8.3), only stored where the attribute is present
    REQUIRE(road.s_to_type.size() == 2);
    REQUIRE(road.s_to_type.at(60.0) == "rural");
    REQUIRE(road.s_to_country.size() == 1);
    REQUIRE(road.s_to_country.at(60.0) == "DE");

    const odr::Lane lane_0 = road.get_lanesection(0.0).get_lane(-1);
    REQUIRE(lane_0.s_to_material.size() == 1);
    const odr::LaneMaterial& material_0 = lane_0.s_to_material.at(0.0);
    REQUIRE(material_0.surface == "asphalt");
    REQUIRE(material_0.friction == 0.9);
    REQUIRE(material_0.roughness == 0.1);
    REQUIRE(lane_0.s_to_speed.at(0.0).max == 80.0);
    REQUIRE(lane_0.s_to_speed.at(0.0).unit == "km/h");
    REQUIRE(lane_0.s_to_access.empty());
    REQUIRE(lane_0.s_to_rule.empty());

    // records are keyed by absolute s (lane section s0 + sOffset)
    const odr::Lane lane_40 = road.get_lanesection(40.0).get_lane(-1);
    REQUIRE(lane_40.s_to_material.size() == 2);
    REQUIRE(lane_40.s_to_material.at(40.0).surface == "concrete");
    REQUIRE(!lane_40.s_to_material.at(40.0).friction.has_value());
    REQUIRE(lane_40.s_to_material.at(60.0).friction == 0.7);

    REQUIRE(lane_40.s_to_speed.size() == 1);
    REQUIRE(lane_40.s_to_speed.at(50.0).max == Catch::Approx(13.9));
    REQUIRE(lane_40.s_to_speed.at(50.0).unit == "m/s");

    REQUIRE(lane_40.s_to_access.size() == 3);
    REQUIRE(lane_40.s_to_access.count(40.0) == 2);
    const auto access_70 = lane_40.s_to_access.find(70.0);
    REQUIRE(access_70 != lane_40.s_to_access.end());
    REQUIRE(access_70->second.rule == "deny");
    REQUIRE(access_70->second.restriction == "pedestrian");

    REQUIRE(lane_40.s_to_rule.size() == 1);
    REQUIRE(lane_40.s_to_rule.begin()->first == 40.0);
    REQUIRE(lane_40.s_to_rule.begin()->second == "no stopping at any time");
}

TEST_CASE("Virtual junctions and junction groups", "[xodr][junction]")
{
    const odr::OpenDriveMap odr_map("junction_virtual_group.xodr");

    const odr::Junction regular = odr_map.get_junction("600");
    REQUIRE(regular.type == "default");
    REQUIRE(regular.id_to_connection.at("0").type == "default");
    REQUIRE(regular.id_to_connection.at("0").contact_point == odr::JunctionConnection::ContactPoint::Start);
    REQUIRE(!regular.id_to_connection.at("0").predecessor.has_value());

    const odr::Junction virtual_junction = odr_map.get_junction("555");
    REQUIRE(virtual_junction.type == "virtual");
    REQUIRE(virtual_junction.id_to_connection.size() == 2);

    const odr::JunctionConnection& conn_3 = virtual_junction.id_to_connection.at("3");
    REQUIRE(conn_3.type == "virtual");
    REQUIRE(conn_3.contact_point == odr::JunctionConnection::ContactPoint::None);
    REQUIRE(conn_3.predecessor.has_value());
    REQUIRE(conn_3.predecessor->element_type == "road");
    REQUIRE(conn_3.predecessor->element_id == "99");
    REQUIRE(conn_3.predecessor->contact_point == "end");
    REQUIRE(!conn_3.predecessor->element_s.has_value());
    REQUIRE(conn_3.successor.has_value());
    REQUIRE(conn_3.successor->element_id == "1");
    REQUIRE(conn_3.successor->element_s == 60.0);
    REQUIRE(conn_3.successor->element_dir == "-");
    REQUIRE(conn_3.lane_links.size() == 1);

    const odr::JunctionConnection& conn_5 = virtual_junction.id_to_connection.at("5");
    REQUIRE(conn_5.predecessor->element_s == 70.0);
    REQUIRE(conn_5.successor->contact_point == "end");

    REQUIRE(odr_map.get_junction_groups().size() == 1);
    const odr::JunctionGroup group = odr_map.get_junction_group("A");
    REQUIRE(group.name == "roundabout_a");
    REQUIRE(group.type == "roundabout");
    REQUIRE(group.junction_ids == std::vector<std::string>{"555", "600"});

    // routing graph must not break on virtual connections
    (void)odr_map.get_routing_graph();
}

TEST_CASE("Object material, parking space, markings, borders, references, tunnels and bridges", "[xodr][objects]")
{
    const odr::OpenDriveMap odr_map("object_extensions.xodr");
    const odr::Road         road = odr_map.get_road("1");

    const odr::RoadObject& parking = road.id_to_object.at("10");
    REQUIRE(parking.parking_space.has_value());
    REQUIRE(parking.parking_space->access == "handicapped");
    REQUIRE(parking.parking_space->restrictions == "max 2h");
    REQUIRE(parking.materials.size() == 1);
    REQUIRE(parking.materials[0].surface == "asphalt");
    REQUIRE(parking.materials[0].friction == 0.8);
    REQUIRE(!parking.materials[0].roughness.has_value());
    REQUIRE(parking.markings.size() == 1);
    REQUIRE(parking.markings[0].side == "front");
    REQUIRE(parking.markings[0].width == 0.12);
    REQUIRE(parking.markings[0].color == "white");
    REQUIRE(parking.markings[0].line_length == 1.0);
    REQUIRE(parking.markings[0].corner_references.empty());

    const odr::RoadObject& island = road.id_to_object.at("11");
    REQUIRE(!island.parking_space.has_value());
    REQUIRE(island.markings.size() == 1);
    REQUIRE(island.markings[0].weight == "bold");
    REQUIRE(island.markings[0].corner_references == std::vector<int>{0, 1});
    REQUIRE(island.borders.size() == 1);
    REQUIRE(island.borders[0].type == "curb");
    REQUIRE(island.borders[0].width == 0.2);
    REQUIRE(island.borders[0].outline_id == 0);
    REQUIRE(!island.borders[0].use_complete_outline);
    REQUIRE(island.borders[0].corner_references == std::vector<int>{1, 2});

    REQUIRE(road.object_references.size() == 1);
    const odr::RoadObjectReference& object_reference = road.object_references[0];
    REQUIRE(object_reference.id == "crosswalk_7");
    REQUIRE(object_reference.s0 == 100.0);
    REQUIRE(object_reference.t0 == 2.0);
    REQUIRE(object_reference.orientation == "+");
    REQUIRE(object_reference.lane_validities.size() == 1);
    REQUIRE(object_reference.lane_validities[0].from_lane == 1);

    const odr::RoadTunnel& tunnel = road.id_to_tunnel.at("t1");
    REQUIRE(tunnel.s0 == 120.0);
    REQUIRE(tunnel.length == 30.0);
    REQUIRE(tunnel.name == "tunnel_a");
    REQUIRE(tunnel.type == "standard");
    REQUIRE(tunnel.lighting == 0.75);
    REQUIRE(tunnel.daylight == 0.25);
    REQUIRE(tunnel.lane_validities.empty());

    const odr::RoadBridge& bridge = road.id_to_bridge.at("b1");
    REQUIRE(bridge.s0 == 160.0);
    REQUIRE(bridge.length == 25.0);
    REQUIRE(bridge.type == "steel");
    REQUIRE(bridge.lane_validities.size() == 1);
    REQUIRE(bridge.lane_validities[0].to_lane == -1);

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

TEST_CASE("Signal dependency, reference, positioning, signal references and controllers", "[xodr][signals]")
{
    const odr::OpenDriveMap odr_map("signal_extensions.xodr");
    const odr::Road         road = odr_map.get_road("1");

    const odr::RoadSignal& traffic_light = road.id_to_signal.at("tl_1");
    REQUIRE(traffic_light.is_dynamic);
    REQUIRE(traffic_light.country_revision == "2017");
    REQUIRE(traffic_light.dependencies.size() == 1);
    REQUIRE(traffic_light.dependencies[0].id == "warn_1");
    REQUIRE(traffic_light.dependencies[0].type == "warning");
    REQUIRE(traffic_light.references.size() == 1);
    REQUIRE(traffic_light.references[0].element_type == "object");
    REQUIRE(traffic_light.references[0].element_id == "stopline_1");
    REQUIRE(traffic_light.references[0].type == "stopLine");
    REQUIRE(traffic_light.position_inertial.has_value());
    REQUIRE(traffic_light.position_inertial->x == 52.0);
    REQUIRE(traffic_light.position_inertial->y == -6.0);
    REQUIRE(traffic_light.position_inertial->z == 5.0);
    REQUIRE(traffic_light.position_inertial->hdg == 3.14);
    REQUIRE(!traffic_light.position_road.has_value());

    const odr::RoadSignal& warning_light = road.id_to_signal.at("warn_1");
    REQUIRE(warning_light.country_revision.empty());
    REQUIRE(warning_light.dependencies.empty());
    REQUIRE(!warning_light.position_inertial.has_value());
    REQUIRE(warning_light.position_road.has_value());
    REQUIRE(warning_light.position_road->road_id == "1");
    REQUIRE(warning_light.position_road->s == 31.0);
    REQUIRE(warning_light.position_road->t == -5.0);
    REQUIRE(warning_light.position_road->z_offset == 2.5);
    REQUIRE(warning_light.position_road->h_offset == 0.1);

    REQUIRE(road.signal_references.size() == 1);
    const odr::RoadSignalReference& signal_reference = road.signal_references[0];
    REQUIRE(signal_reference.id == "sign_1");
    REQUIRE(signal_reference.s0 == 70.0);
    REQUIRE(signal_reference.t0 == 4.0);
    REQUIRE(signal_reference.orientation == "-");
    REQUIRE(signal_reference.lane_validities.size() == 1);
    REQUIRE(signal_reference.lane_validities[0].from_lane == 1);

    REQUIRE(odr_map.get_controllers().size() == 2);
    const odr::Controller controller = odr_map.get_controller("ctrl_1");
    REQUIRE(controller.name == "main controller");
    REQUIRE(controller.sequence == 1u);
    REQUIRE(controller.controls.size() == 2);
    REQUIRE(controller.controls[0].signal_id == "tl_1");
    REQUIRE(controller.controls[1].signal_id == "warn_1");
    REQUIRE(!odr_map.get_controller("ctrl_2").sequence.has_value());
    REQUIRE(odr_map.get_controller("ctrl_2").controls.empty());

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

TEST_CASE("Road mark explicit lines and sway", "[xodr][roadmark]")
{
    const odr::OpenDriveMap odr_map("roadmark_explicit_sway.xodr", false);
    const odr::Road         road = odr_map.get_road("1");
    const odr::Lane         lane = road.get_lanesection(0.0).get_lane(-1);

    // explicit lines are not repeated: two road marks within [0, 50)
    const std::vector<odr::RoadMark> explicit_roadmarks = lane.get_roadmarks(0.0, 50.0);
    REQUIRE(explicit_roadmarks.size() == 2);
    REQUIRE(explicit_roadmarks[0].s_start == 0.0);
    REQUIRE(explicit_roadmarks[0].s_end == 3.0);
    REQUIRE(explicit_roadmarks[0].width == 0.2); // falls back to <roadMark> @width
    REQUIRE(explicit_roadmarks[1].s_start == 10.0);
    REQUIRE(explicit_roadmarks[1].s_end == 12.0);
    REQUIRE(explicit_roadmarks[1].width == 0.15);
    REQUIRE(explicit_roadmarks[1].t_offset == 0.1);

    // sway shifts the solid road mark from s=50: constant 0.5 until s=70, then linearly back to 0 at s=90
    const std::vector<odr::RoadMark> sway_roadmarks = lane.get_roadmarks(50.0, 100.0);
    REQUIRE(sway_roadmarks.size() == 1);
    const odr::RoadMark& swayed = sway_roadmarks[0];
    REQUIRE(swayed.sway.evaluate(60.0, 0.0, false) == Catch::Approx(0.5));
    REQUIRE(swayed.sway.evaluate(80.0, 0.0, false) == Catch::Approx(0.25));
    REQUIRE(swayed.sway.evaluate(90.0, 0.0, false) == Catch::Approx(0.0).margin(1e-9));

    const odr::Mesh3D mesh = road.get_roadmark_mesh(lane, swayed, 0.1);
    REQUIRE(!mesh.vertices.empty());
    // first vertex pair at s=50: lane border at t=-3.5, half width 0.06, sway 0.5
    REQUIRE(mesh.vertices[0][0] == Catch::Approx(50.0));
    REQUIRE(mesh.vertices[0][1] == Catch::Approx(-3.5 + 0.06 + 0.5));
    REQUIRE(mesh.vertices[1][1] == Catch::Approx(-3.5 - 0.06 + 0.5));

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}

TEST_CASE("Road mark line color", "[xodr][roadmark]")
{
    const odr::OpenDriveMap odr_map("roadmark_color.xodr", false);
    const odr::Road         road = odr_map.get_road("1");
    const odr::LaneSection  lanesec = road.get_lanesection(0.0);

    // <type><line @color> supersedes the <roadMark> @color, otherwise the line inherits it
    const std::vector<odr::RoadMark> center_roadmarks = lanesec.get_lane(0).get_roadmarks(0.0, 20.0);
    REQUIRE(center_roadmarks.size() == 2);
    for (const odr::RoadMark& roadmark : center_roadmarks)
    {
        if (roadmark.t_offset > 0)
            REQUIRE(roadmark.color == "yellow");
        else
            REQUIRE(roadmark.color == "white");
    }

    const std::vector<odr::RoadMark> right_roadmarks = lanesec.get_lane(-1).get_roadmarks(0.0, 20.0);
    REQUIRE(right_roadmarks.size() == 1);
    REQUIRE(right_roadmarks[0].color == "blue");

    const odr::RoadmarksMesh roadmarks_mesh = odr_map.get_road_network_mesh(0.1).roadmarks_mesh;
    std::set<std::string>    mesh_colors;
    for (std::size_t vert_idx = 0; vert_idx < roadmarks_mesh.vertices.size(); vert_idx++)
        mesh_colors.insert(roadmarks_mesh.get_roadmark_color(vert_idx));
    REQUIRE(mesh_colors == std::set<std::string>{"yellow", "white", "blue"});
}

TEST_CASE("Surface CRG references and additional data", "[xodr][crg][additional_data]")
{
    const odr::OpenDriveMap odr_map("crg_additional_data.xodr");

    // root and header
    REQUIRE(odr_map.header.additional_data.user_data.size() == 1);
    REQUIRE(odr_map.header.additional_data.user_data[0].code == "generator");
    REQUIRE(odr_map.header.additional_data.user_data[0].value == "unit test");
    REQUIRE(odr_map.additional_data.includes == std::vector<std::string>{"extra_signals.xml"});
    REQUIRE(odr_map.additional_data.data_quality.has_value());
    const odr::DataQuality& data_quality = *odr_map.additional_data.data_quality;
    REQUIRE(data_quality.error->xy_absolute == 0.05);
    REQUIRE(data_quality.error->z_relative == 0.02);
    REQUIRE(data_quality.raw_data->source == "sensor");
    REQUIRE(data_quality.raw_data->post_processing == "cleaned");
    REQUIRE(data_quality.raw_data->post_processing_comment == "filtered");

    // road, lane section, lane, object, signal
    const odr::Road road = odr_map.get_road("1");
    REQUIRE(road.additional_data.user_data.size() == 1);
    REQUIRE(road.additional_data.user_data[0].code == "road_note");
    const odr::LaneSection lanesection = road.get_lanesection(0.0);
    REQUIRE(lanesection.additional_data.user_data[0].code == "section_note");
    const odr::Lane lane = lanesection.get_lane(-1);
    REQUIRE(lane.additional_data.user_data.size() == 1);
    REQUIRE(lane.additional_data.user_data[0].value.empty());
    REQUIRE(lane.additional_data.user_data[0].content.find("<texture name=\"asphalt_01\" scale=\"2\"") != std::string::npos);
    REQUIRE(lanesection.get_lane(0).additional_data.empty());
    REQUIRE(road.id_to_object.at("o1").additional_data.user_data[0].value == "pole.fbx");
    REQUIRE(road.id_to_signal.at("s1").additional_data.includes == std::vector<std::string>{"sign_details.xml"});

    // road surface CRG
    REQUIRE(road.surface_crgs.size() == 2);
    const odr::SurfaceCRG& crg_elevation = road.surface_crgs[0];
    REQUIRE(crg_elevation.file == "road_1.crg");
    REQUIRE(crg_elevation.mode == "attached");
    REQUIRE(crg_elevation.purpose == "elevation");
    REQUIRE(crg_elevation.s_start == 0.0);
    REQUIRE(crg_elevation.s_end == 100.0);
    REQUIRE(crg_elevation.orientation == "same");
    REQUIRE(crg_elevation.t_offset == 0.5);
    REQUIRE(crg_elevation.z_scale == 0.8);
    REQUIRE(crg_elevation.s_offset == 0.0);
    const odr::SurfaceCRG& crg_friction = road.surface_crgs[1];
    REQUIRE(crg_friction.purpose == "friction");
    REQUIRE(crg_friction.mode == "genuine");
    REQUIRE(crg_friction.h_offset == 0.1);
    REQUIRE(crg_friction.z_scale == 1.0);
    REQUIRE(!crg_friction.orientation.has_value());

    // junction surface CRG and user data
    const odr::Junction junction = odr_map.get_junction("100");
    REQUIRE(junction.surface_crgs.size() == 1);
    REQUIRE(junction.surface_crgs[0].file == "junction.crg");
    REQUIRE(junction.surface_crgs[0].mode == "global");
    REQUIRE(junction.surface_crgs[0].purpose == "elevation");
    REQUIRE(junction.surface_crgs[0].z_offset == 1.5);
    REQUIRE(!junction.surface_crgs[0].s_start.has_value());
    REQUIRE(junction.additional_data.user_data[0].code == "junction_note");
}

TEST_CASE("Include files are merged where they appear", "[xodr][include]")
{
    const odr::OpenDriveMap odr_map("include_main.xodr");

    // <include> elements stay available as additional data, including those merged from included files (@file as written there)
    REQUIRE(odr_map.additional_data.includes == std::vector<std::string>{"road_3.xml", "include/roads.xml", "include/missing.xml"});

    // road 1: planView geometries from the included <planView>
    const odr::Road road_1 = odr_map.get_road("1");
    REQUIRE(road_1.ref_line.s0_to_geometry.size() == 2);
    REQUIRE(road_1.ref_line.s0_to_geometry.count(10.0) == 1);
    const odr::Vec3D pt_1 = road_1.ref_line.get_xyz(25.0);
    REQUIRE(pt_1[0] == Catch::Approx(25.0));
    REQUIRE(pt_1[1] == Catch::Approx(0.0));

    // road 2: included from include/roads.xml, its lanes from the nested include/lane_2.xml
    REQUIRE(odr_map.id_to_road.size() == 3);
    const odr::Road        road_2 = odr_map.get_road("2");
    const odr::LaneSection lanesection_2 = road_2.get_lanesection(0.0);
    REQUIRE(lanesection_2.id_to_lane.size() == 3);
    REQUIRE(lanesection_2.get_lane(-2).type == "sidewalk");
    REQUIRE(road_2.get_xyz(10.0, -4.0, 0.0)[0] == Catch::Approx(40.0));

    // road 3: root element <road> inserted as a whole, the recursive include in it is skipped
    const odr::Road road_3 = odr_map.get_road("3");
    REQUIRE(road_3.length == 15.0);
    REQUIRE(road_3.additional_data.includes == std::vector<std::string>{"roads.xml"});
    REQUIRE(road_3.get_lanesection(0.0).id_to_lane.size() == 2);
}

TEST_CASE("Railroad switches and stations", "[xodr][railroad]")
{
    const odr::OpenDriveMap odr_map("railroad.xodr");

    const odr::Road road = odr_map.get_road("1");
    REQUIRE(road.id_to_switch.size() == 2);

    const odr::RailroadSwitch& sw1 = road.id_to_switch.at("sw1");
    REQUIRE(sw1.road_id == "1");
    REQUIRE(sw1.name == "W1");
    REQUIRE(sw1.position == "dynamic");
    REQUIRE(sw1.main_track.id == "1");
    REQUIRE(sw1.main_track.s == 50.0);
    REQUIRE(sw1.main_track.dir == "+");
    REQUIRE(sw1.side_track.id == "2");
    REQUIRE(sw1.side_track.s == 0.0);
    REQUIRE(sw1.side_track.dir == "-");
    REQUIRE(sw1.partner.has_value());
    REQUIRE(sw1.partner->id == "sw2");
    REQUIRE(sw1.partner->name == "W2");

    // direction given as @position as in the 13.2 attribute table
    const odr::RailroadSwitch& sw3 = road.id_to_switch.at("sw3");
    REQUIRE(sw3.position == "static");
    REQUIRE(sw3.main_track.dir == "-");
    REQUIRE(sw3.side_track.s == 40.0);
    REQUIRE(sw3.side_track.dir == "+");
    REQUIRE(!sw3.partner.has_value());

    REQUIRE(road.get_lanesection(0.0).get_lane(-1).type == "rail");
    REQUIRE(odr_map.get_road("2").get_lanesection(0.0).get_lane(-1).type == "tram");
    REQUIRE(odr_map.get_road("2").id_to_switch.empty());

    REQUIRE(odr_map.get_stations().size() == 1);
    const odr::Station station = odr_map.get_station("st1");
    REQUIRE(station.name == "Central");
    REQUIRE(station.type == "large");
    REQUIRE(station.platforms.size() == 2);
    REQUIRE(station.platforms[0].id == "p1");
    REQUIRE(station.platforms[0].name == "Platform 1");
    REQUIRE(station.platforms[0].segments.size() == 2);
    REQUIRE(station.platforms[0].segments[0].road_id == "1");
    REQUIRE(station.platforms[0].segments[0].s_start == 10.0);
    REQUIRE(station.platforms[0].segments[0].s_end == 40.0);
    REQUIRE(station.platforms[0].segments[0].side == "right");
    REQUIRE(station.platforms[0].segments[1].road_id == "2");
    REQUIRE(station.platforms[0].segments[1].side == "left");
    REQUIRE(station.platforms[1].segments.empty());
}

TEST_CASE("Object repeat length and radius interpolation", "[xodr][objects]")
{
    const odr::OpenDriveMap odr_map("object_repeat.xodr");
    const odr::Road&        road = odr_map.get_road("1");

    const odr::RoadObject& box = road.id_to_object.at("box");
    REQUIRE(box.repeats.size() == 1);
    REQUIRE(box.repeats[0].length_start == 2.0);
    REQUIRE(box.repeats[0].length_end == 6.0);
    REQUIRE(std::isnan(box.repeats[0].radius_start));
    REQUIRE(std::isnan(box.repeats[0].radius_end));

    // instances at s = 0, 50, 100 with length 2, 4, 6 (8 vertices per box)
    const odr::Mesh3D box_mesh = road.get_road_object_mesh(box, 0.1);
    REQUIRE(box_mesh.vertices.size() == 3 * 8);
    const std::array<double, 3> s_instances = {0.0, 50.0, 100.0};
    const std::array<double, 3> expected_lengths = {2.0, 4.0, 6.0};
    for (std::size_t i = 0; i < 3; i++)
    {
        double x_min = INFINITY, x_max = -INFINITY;
        for (std::size_t v = i * 8; v < (i + 1) * 8; v++)
        {
            x_min = std::min(x_min, box_mesh.vertices[v][0]);
            x_max = std::max(x_max, box_mesh.vertices[v][0]);
        }
        REQUIRE(x_max - x_min == Catch::Approx(expected_lengths[i]));
        REQUIRE(0.5 * (x_max + x_min) == Catch::Approx(s_instances[i]));
    }

    const odr::RoadObject& pole = road.id_to_object.at("pole");
    REQUIRE(pole.repeats[0].radius_start == 0.1);
    REQUIRE(pole.repeats[0].radius_end == 0.5);

    // cylinders at s = 0, 50, 100 with radius 0.1, 0.3, 0.5
    const odr::Mesh3D           pole_mesh = road.get_road_object_mesh(pole, 0.1);
    const std::array<double, 3> expected_radii = {0.1, 0.3, 0.5};
    std::array<double, 3>       max_radii = {0, 0, 0};
    for (const odr::Vec3D& pt : pole_mesh.vertices)
    {
        REQUIRE(pt[1] == Catch::Approx(-5.0).margin(0.6));
        const std::size_t i = static_cast<std::size_t>(std::round(pt[0] / 50.0));
        REQUIRE(i < 3);
        max_radii[i] = std::max(max_radii[i], std::hypot(pt[0] - s_instances[i], pt[1] + 5.0));
    }
    for (std::size_t i = 0; i < 3; i++)
        REQUIRE(max_radii[i] == Catch::Approx(expected_radii[i]));

    // continuous object without width uses the interpolated diameter
    const odr::RoadObject& pipe = road.id_to_object.at("pipe");
    const odr::Mesh3D      pipe_mesh = road.get_road_object_mesh(pipe, 0.1);
    REQUIRE(!pipe_mesh.vertices.empty());
    double t_min_start = INFINITY, t_max_start = -INFINITY, t_min_end = INFINITY, t_max_end = -INFINITY;
    for (const odr::Vec3D& pt : pipe_mesh.vertices)
    {
        if (pt[0] == Catch::Approx(0.0).margin(1e-6))
        {
            t_min_start = std::min(t_min_start, pt[1]);
            t_max_start = std::max(t_max_start, pt[1]);
        }
        if (pt[0] == Catch::Approx(20.0).margin(1e-6))
        {
            t_min_end = std::min(t_min_end, pt[1]);
            t_max_end = std::max(t_max_end, pt[1]);
        }
    }
    REQUIRE(t_max_start - t_min_start == Catch::Approx(0.4));
    REQUIRE(t_max_end - t_min_end == Catch::Approx(0.8));
}

TEST_CASE("Signal mesh at physical position", "[xodr][signals]")
{
    const auto get_mesh_center = [](const odr::Mesh3D& mesh)
    {
        odr::Vec3D center{0, 0, 0};
        for (const odr::Vec3D& pt : mesh.vertices)
            center = odr::add(center, pt);
        return odr::mut(1.0 / mesh.vertices.size(), center);
    };

    for (const bool center_map : {false, true})
    {
        const odr::OpenDriveMap odr_map("signal_physical_position.xodr", center_map);
        const odr::Road         road = odr_map.get_road("1");
        // origin of the file's inertial coordinate system in the map frame
        const odr::Vec3D origin = odr::sub(road.get_xyz(0.0, 0.0, 0.0), odr::Vec3D{100.0, 200.0, 0.0});

        const auto require_signal_center = [&](const std::string& signal_id, const odr::Vec3D& expected)
        {
            const odr::RoadSignal& road_signal = road.id_to_signal.at(signal_id);
            const odr::Mesh3D      mesh = odr_map.get_road_signal_mesh(road_signal);
            REQUIRE(mesh.vertices.size() == 8);
            const odr::Vec3D center = odr::sub(get_mesh_center(mesh), origin);
            REQUIRE(center[0] == Catch::Approx(expected[0]));
            REQUIRE(center[1] == Catch::Approx(expected[1]));
            REQUIRE(center[2] == Catch::Approx(expected[2]));
            // the mesh stays attached to the logical position
            for (const odr::Vec2D& st : mesh.st_coordinates)
            {
                REQUIRE(st[0] == road_signal.s0);
                REQUIRE(st[1] == road_signal.t0);
            }
        };

        require_signal_center("inertial", {110.0, 194.0, 5.5});
        require_signal_center("road_same", {131.0, 195.0, 3.0});
        require_signal_center("road_other", {140.0, 221.0, 3.5});
        require_signal_center("road_missing", {170.0, 196.0, 2.5}); // unknown road: logical position
        require_signal_center("logical", {160.0, 196.0, 2.5});

        // Road::get_road_signal_mesh resolves positionRoad only on its own road
        const odr::Mesh3D road_other_mesh = road.get_road_signal_mesh(road.id_to_signal.at("road_other"));
        const odr::Vec3D  road_other_center = odr::sub(get_mesh_center(road_other_mesh), origin);
        REQUIRE(road_other_center[0] == Catch::Approx(120.0));
        REQUIRE(road_other_center[1] == Catch::Approx(196.0));

        REQUIRE(odr_map.get_road_network_mesh(0.1).road_signals_mesh.vertices.size() == 5 * 8);
    }
}

TEST_CASE("Signal mesh orientation", "[xodr][signals]")
{
    const odr::OpenDriveMap odr_map("signal_orientation.xodr");
    const odr::Road         road = odr_map.get_road("1");

    // the box is tilted by @pitch around its facing axis, so the top leans along +s for "+" and along -s for "-"
    const auto get_lean_s = [&](const std::string& signal_id, const double s_base)
    {
        const odr::Mesh3D mesh = odr_map.get_road_signal_mesh(road.id_to_signal.at(signal_id));
        REQUIRE(mesh.vertices.size() == 8);
        odr::Vec3D center{0, 0, 0};
        for (const odr::Vec3D& pt : mesh.vertices)
            center = odr::add(center, pt);
        center = odr::mut(1.0 / mesh.vertices.size(), center);
        return center[0] - road.get_xyz(s_base, 0.0, 0.0)[0];
    };

    const double lean = 0.5 * std::sin(0.3);
    REQUIRE(get_lean_s("plus", 20.0) == Catch::Approx(lean));
    REQUIRE(get_lean_s("minus", 40.0) == Catch::Approx(-lean));
    REQUIRE(get_lean_s("minus_road", 80.0) == Catch::Approx(-lean)); // positionRoad @hOffset is relative to @orientation too
    REQUIRE(get_lean_s("none", 70.0) == Catch::Approx(lean));        // "none": relative to the reference line
}
