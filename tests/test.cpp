#include "OpenDriveMap.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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
