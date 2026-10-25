#include "Lane.h"
#include "Fmt.hpp"

#include <algorithm>
#include <iterator>
#include <sstream>
#include <utility>

namespace odr
{

namespace
{
// lines {t_offset, is_broken} of a <roadMark> @type without a <type> definition; double lines are spaced by the line width.
// OpenDRIVE 1.6.1 e_roadMarkType: the keywords run from inside to outside, for the center lane from left to right
std::vector<std::pair<double, bool>> get_default_roadmark_lines(const std::string& type, const double width, const int lane_id)
{
    std::istringstream       type_stream(type);
    std::vector<std::string> keywords{std::istream_iterator<std::string>(type_stream), std::istream_iterator<std::string>()};
    const auto               is_line_keyword = [](const std::string& keyword) { return keyword == "solid" || keyword == "broken"; };

    if (keywords.size() == 2 && is_line_keyword(keywords[0]) && is_line_keyword(keywords[1]))
    {
        // inside is towards t = 0, i.e. -t for left lanes; the center lane and right lanes start at +t
        const double t_first = lane_id > 0 ? -width : width;
        return {{t_first, keywords[0] == "broken"}, {-t_first, keywords[1] == "broken"}};
    }
    return {{0.0, type == "broken"}};
}
} // namespace

HeightOffset::HeightOffset(double inner, double outer) : inner(inner), outer(outer) {}

LaneMaterial::LaneMaterial(std::string surface, std::optional<double> friction, std::optional<double> roughness) :
    surface(surface), friction(friction), roughness(roughness)
{
}

LaneSpeed::LaneSpeed(double max, std::string unit) : max(max), unit(unit) {}

LaneAccess::LaneAccess(std::string rule, std::string restriction) : rule(rule), restriction(restriction) {}

LaneKey::LaneKey(std::string road_id, double lanesection_s0, int lane_id) : road_id(road_id), lanesection_s0(lanesection_s0), lane_id(lane_id) {}

std::string LaneKey::to_string() const
{
    return strfmt("%s/%.17g/%d", this->road_id.c_str(), this->lanesection_s0, this->lane_id);
}

Lane::Lane(std::string road_id, double lanesection_s0, int id, bool level, std::string type) :
    key(road_id, lanesection_s0, id), id(id), level(level), type(type)
{
}

std::vector<RoadMark> Lane::get_roadmarks(const double s_start, const double s_end) const
{
    if ((s_start == s_end) || this->roadmark_groups.empty())
        return {};

    auto s_start_rm_iter =
        std::upper_bound(this->roadmark_groups.begin(),
                         this->roadmark_groups.end(),
                         s_start,
                         [](const double& s, const RoadMarkGroup& rmg) -> bool { return s < (rmg.lanesection_s0 + rmg.s_offset); });
    if (s_start_rm_iter != this->roadmark_groups.begin())
        s_start_rm_iter--;

    auto s_end_rm_iter = std::lower_bound(this->roadmark_groups.begin(),
                                          this->roadmark_groups.end(),
                                          s_end,
                                          [](const RoadMarkGroup& rmg, const double& s) -> bool { return (rmg.lanesection_s0 + rmg.s_offset) < s; });

    std::vector<RoadMark> roadmarks;
    for (auto rm_group_iter = s_start_rm_iter; rm_group_iter != s_end_rm_iter; rm_group_iter++)
    {
        const RoadMarkGroup& roadmark_group = *rm_group_iter;

        const double roadmark_group_s0 = roadmark_group.lanesection_s0 + roadmark_group.s_offset;
        const double s_start_roadmark_group = std::max(roadmark_group_s0, s_start);
        const double s_end_roadmark_group = (std::next(rm_group_iter) == s_end_rm_iter)
                                                ? s_end
                                                : std::min(std::next(rm_group_iter)->lanesection_s0 + std::next(rm_group_iter)->s_offset, s_end);

        const std::size_t num_roadmarks_before_group = roadmarks.size();

        double width = roadmark_group.weight == "bold" ? ROADMARK_WEIGHT_BOLD_WIDTH : ROADMARK_WEIGHT_STANDARD_WIDTH;
        if (!roadmark_group.roadmark_explicit_lines.empty())
        {
            // explicit line @width supersedes the <roadMark> @width
            const double group_width = roadmark_group.width > 0 ? roadmark_group.width : width;
            // OpenDRIVE 1.6.1, 9.6.2: explicit lines are not repeated
            for (const RoadMarksLine& roadmarks_line : roadmark_group.roadmark_explicit_lines)
            {
                const double s0_roadmarks_line = roadmarks_line.group_s0 + roadmarks_line.s_offset;
                const double s_start_single_roadmark = std::max(s0_roadmarks_line, s_start_roadmark_group);
                const double s_end_single_roadmark = std::min(s0_roadmarks_line + roadmarks_line.length, s_end_roadmark_group);
                if (s_end_single_roadmark <= s_start_single_roadmark)
                    continue;

                roadmarks.push_back(RoadMark(this->key.road_id,
                                             this->key.lanesection_s0,
                                             this->id,
                                             roadmarks_line.group_s0,
                                             s_start_single_roadmark,
                                             s_end_single_roadmark,
                                             roadmarks_line.t_offset,
                                             roadmarks_line.width > 0 ? roadmarks_line.width : group_width,
                                             roadmark_group.type + roadmarks_line.name,
                                             roadmarks_line.color));
            }
        }
        else if (roadmark_group.roadmark_lines.empty())
        {
            // OpenDRIVE 1.6.1, 9.6: "none" has no visible road mark
            if (roadmark_group.type == "none")
                continue;

            if (roadmark_group.width > 0)
                width = roadmark_group.width;

            for (const auto& [t_offset, is_broken] : get_default_roadmark_lines(roadmark_group.type, width, this->id))
            {
                if (!is_broken)
                {
                    roadmarks.push_back(RoadMark(this->key.road_id,
                                                 this->key.lanesection_s0,
                                                 this->id,
                                                 roadmark_group_s0,
                                                 s_start_roadmark_group,
                                                 s_end_roadmark_group,
                                                 t_offset,
                                                 width,
                                                 roadmark_group.type,
                                                 roadmark_group.color));
                    continue;
                }

                for (double s_start_single_roadmark = roadmark_group_s0; s_start_single_roadmark < s_end_roadmark_group;
                     s_start_single_roadmark += (ROADMARK_BROKEN_LENGTH + ROADMARK_BROKEN_SPACE))
                {
                    const double s_end_single_roadmark = std::min(s_end_roadmark_group, s_start_single_roadmark + ROADMARK_BROKEN_LENGTH);
                    if (s_end_single_roadmark <= s_start_roadmark_group)
                        continue;
                    roadmarks.push_back(RoadMark(this->key.road_id,
                                                 this->key.lanesection_s0,
                                                 this->id,
                                                 roadmark_group_s0,
                                                 std::max(s_start_single_roadmark, s_start_roadmark_group),
                                                 s_end_single_roadmark,
                                                 t_offset,
                                                 width,
                                                 roadmark_group.type,
                                                 roadmark_group.color));
                }
            }
        }
        else
        {
            for (const RoadMarksLine& roadmarks_line : roadmark_group.roadmark_lines)
            {
                if (roadmarks_line.width > 0)
                    width = roadmarks_line.width;

                if ((roadmarks_line.length + roadmarks_line.space) == 0)
                    continue;

                const double s0_roadmarks_line = roadmarks_line.group_s0 + roadmarks_line.s_offset;
                for (double s_start_single_roadmark = s0_roadmarks_line; s_start_single_roadmark < s_end_roadmark_group;
                     s_start_single_roadmark += (roadmarks_line.length + roadmarks_line.space))
                {
                    const double s_end_single_roadmark = std::min(s_end, s_start_single_roadmark + roadmarks_line.length);
                    roadmarks.push_back(RoadMark(this->key.road_id,
                                                 this->key.lanesection_s0,
                                                 this->id,
                                                 roadmarks_line.group_s0,
                                                 s_start_single_roadmark,
                                                 s_end_single_roadmark,
                                                 roadmarks_line.t_offset,
                                                 width,
                                                 roadmark_group.type + roadmarks_line.name,
                                                 roadmarks_line.color));
                }
            }
        }

        for (auto rm_iter = std::next(roadmarks.begin(), num_roadmarks_before_group); rm_iter != roadmarks.end(); rm_iter++)
            rm_iter->sway = roadmark_group.sway;
    }

    return roadmarks;
}

} // namespace odr
