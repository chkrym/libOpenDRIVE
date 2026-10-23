#include "OpenDriveMap.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

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

TEST_CASE("Poly3 geometry", "[xodr][poly3]")
{
    odr::OpenDriveMap odr_map("poly3.xodr", false);

    // straight line: start point is offset by v = a, heading 90 degrees
    const odr::Road  straight = odr_map.get_road("1");
    const odr::Vec3D start = straight.get_xyz(0.0, 0.0, 0.0);
    REQUIRE(start[0] == Catch::Approx(8.0));
    REQUIRE(start[1] == Catch::Approx(0.0).margin(1e-9));
    const odr::Vec3D end = straight.get_xyz(50.0, 0.0, 0.0);
    REQUIRE(end[0] == Catch::Approx(8.0));
    REQUIRE(end[1] == Catch::Approx(50.0));

    // curved: end point lies on v(u) and the reference line length matches @length
    const odr::Road  curved = odr_map.get_road("2");
    const odr::Vec3D curved_end = curved.get_xyz(35.0, 0.0, 0.0);
    const double     u = curved_end[0];
    REQUIRE(curved_end[1] == Catch::Approx(0.1 * u + 0.002 * u * u - 0.00003 * u * u * u));

    double     arc_length = 0.0;
    odr::Vec3D prev = curved.get_xyz(0.0, 0.0, 0.0);
    for (int i = 1; i <= 3500; i++)
    {
        const odr::Vec3D pt = curved.get_xyz(i * 0.01, 0.0, 0.0);
        arc_length += std::hypot(pt[0] - prev[0], pt[1] - prev[1]);
        prev = pt;
    }
    REQUIRE(arc_length == Catch::Approx(35.0).epsilon(1e-4));

    REQUIRE(!odr_map.get_road_network_mesh(0.1).get_mesh().vertices.empty());
}
