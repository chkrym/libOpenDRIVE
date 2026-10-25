#include "Road.h"
#include "Fmt.hpp"
#include "Lane.h"
#include "RefLine.h"
#include "RoadMark.h"
#include "Utils.hpp"

#include "earcut.hpp"
#include "pugixml.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>

namespace odr
{

double Crossfall::get_crossfall(const double s, const bool on_left_side) const
{
    if (this->segments.size() > 0)
    {
        auto target_poly_iter = this->segments.upper_bound(s);
        if (target_poly_iter != this->segments.begin())
            target_poly_iter--;

        Side side = Side::Both; // applicable side of the road
        if (this->sides.find(target_poly_iter->first) != this->sides.end())
            side = this->sides.at(target_poly_iter->first);

        if (on_left_side && side == Side::Right)
            return 0;
        else if (!on_left_side && side == Side::Left)
            return 0;

        return target_poly_iter->second.evaluate(s);
    }

    return 0;
}

RoadLink::RoadLink(std::string id, Type type, ContactPoint contact_point) : id(id), type(type), contact_point(contact_point) {}

RoadNeighbor::RoadNeighbor(std::string id, std::string side, std::string direction) : id(id), side(side), direction(direction) {}

SpeedRecord::SpeedRecord(std::string max, std::string unit) : max(max), unit(unit) {}

std::vector<LaneSection> Road::get_lanesections() const
{
    return get_map_values(this->s_to_lanesection);
}
std::vector<RoadObject> Road::get_road_objects() const
{
    return get_map_values(this->id_to_object);
}

std::vector<RoadSignal> Road::get_road_signals() const
{
    return get_map_values(this->id_to_signal);
}

Road::Road(std::string id, double length, std::string junction, std::string name, bool left_hand_traffic) :
    length(length), id(id), junction(junction), name(name), left_hand_traffic(left_hand_traffic), ref_line(length)
{
}

double Road::get_lanesection_s0(const double s) const
{
    if (this->s_to_lanesection.empty())
        return NAN;

    auto s_lanesec_iter = this->s_to_lanesection.upper_bound(s);
    if (s_lanesec_iter != this->s_to_lanesection.begin())
        s_lanesec_iter--;
    const LaneSection& lanesec = s_lanesec_iter->second;

    if (s < lanesec.s0 || s > this->get_lanesection_end(lanesec))
        return NAN;

    return lanesec.s0;
}

LaneSection Road::get_lanesection(const double s) const
{
    const double lanesec_s0 = this->get_lanesection_s0(s);
    if (std::isnan(lanesec_s0))
        throw std::runtime_error("no valid lanesection");
    return this->s_to_lanesection.at(lanesec_s0);
}

double Road::get_lanesection_end(const LaneSection& lanesection) const
{
    if (lanesection.road_id != this->id)
        log::error("LaneSection %f in Road #%s, not in Road #%s", lanesection.s0, lanesection.road_id.c_str(), this->id.c_str());
    return this->get_lanesection_end(lanesection.s0);
}

double Road::get_lanesection_end(const double lanesection_s0) const
{
    auto s_lanesec_iter = this->s_to_lanesection.find(lanesection_s0);
    if (s_lanesec_iter == this->s_to_lanesection.end())
        return NAN;

    const bool is_last = (s_lanesec_iter == std::prev(this->s_to_lanesection.end()));
    if (is_last)
        return this->length;

    const double next_s0 = std::next(s_lanesec_iter)->first;
    return std::nextafter(next_s0, std::numeric_limits<double>::lowest()); // should be within lane section
}

double Road::get_lanesection_length(const LaneSection& lanesection) const
{
    const double s_end = this->get_lanesection_end(lanesection);
    return s_end - lanesection.s0;
}

double Road::get_lanesection_length(const double lanesection_s0) const
{
    const double s_end = this->get_lanesection_end(lanesection_s0);
    return s_end - lanesection_s0;
}

Vec3D Road::get_xyz(const double s, const double t, const double h, Vec3D* _e_s, Vec3D* _e_t, Vec3D* _e_h) const
{
    const Vec3D  s_vec = this->ref_line.derivative(s);
    const double theta = this->superelevation.evaluate(s);

    const Vec3D e_s = normalize(s_vec);
    const Vec3D e_t = normalize(Vec3D{std::cos(theta) * -e_s[1] + std::sin(theta) * -e_s[2] * e_s[0],
                                      std::cos(theta) * e_s[0] + std::sin(theta) * -e_s[2] * e_s[1],
                                      std::sin(theta) * (e_s[0] * e_s[0] + e_s[1] * e_s[1])});
    const Vec3D e_h = normalize(crossProduct(s_vec, e_t));
    const Vec3D p0 = this->ref_line.get_xyz(s);
    const Mat3D trans_mat{{{e_t[0], e_h[0], p0[0]}, {e_t[1], e_h[1], p0[1]}, {e_t[2], e_h[2], p0[2]}}};

    const Vec3D xyz = MatVecMultiplication(trans_mat, Vec3D{t, h, 1});

    if (_e_s)
        *_e_s = e_s;
    if (_e_t)
        *_e_t = e_t;
    if (_e_h)
        *_e_h = e_h;

    return xyz;
}

Vec3D Road::get_surface_pt(double s, const double t, Vec3D* vn) const
{
    odr::check_and_repair(
        s >= 0, [&]() { s = 0; }, "Road #%s: get_surface_pt for s < 0 invalid (s=%f), getting for s=0", this->id.c_str(), s);
    odr::check_and_repair(
        s <= this->length,
        [&]() { s = this->length; },
        "Road #%s: get_surface_pt for s > length invalid (s=%f, length=%f), getting for s=length",
        this->id.c_str(),
        s,
        this->length);

    const double lanesection_s0 = this->get_lanesection_s0(s);
    if (std::isnan(lanesection_s0))
    {
        throw std::runtime_error(strfmt("cannot get road surface pt, no lane section for s %.3f, road length: %.3f", s, this->length));
    }

    const LaneSection& lanesection = this->s_to_lanesection.at(lanesection_s0);
    const Lane&        lane = lanesection.get_lane(s, t);
    const Lane&        inner_neighbor_lane = lanesection.get_lane(next_towards_zero(lane.id));
    const double       t_inner_brdr = inner_neighbor_lane.outer_border.evaluate(s);
    double             h_t = 0;
    double             dh_dt = 0;

    if (lane.level)
    {
        // OpenDRIVE 1.6.1, 9.5.8: a lane with @level=true is excluded from both superelevation and road shape and "stays on the
        // same height as the inner connecting lane". Several outer lanes may be level, so the height is taken at the inner
        // border of the innermost lane of the consecutive level lanes.
        int level_base_lane_id = lane.id;
        while (level_base_lane_id != 0)
        {
            const int  next_inner_lane_id = next_towards_zero(level_base_lane_id);
            const auto next_inner_lane_iter = lanesection.id_to_lane.find(next_inner_lane_id);
            if (next_inner_lane_id == 0 || next_inner_lane_iter == lanesection.id_to_lane.end() || !next_inner_lane_iter->second.level)
                break;
            level_base_lane_id = next_inner_lane_id;
        }
        const double t_level_base =
            (level_base_lane_id == lane.id) ? t_inner_brdr : lanesection.get_lane(next_towards_zero(level_base_lane_id)).outer_border.evaluate(s);

        const double h_level_base =
            -std::tan(this->crossfall.get_crossfall(s, (lane.id > 0))) * std::abs(t_level_base) + this->get_lateral_shape(s, t_level_base);
        dh_dt = -std::tan(this->superelevation.evaluate(s)); // cancel out superelevation, e_t is rolled up by superelevation
        h_t = h_level_base + dh_dt * (t - t_level_base);
    }
    else
    {
        h_t = -std::tan(this->crossfall.get_crossfall(s, (lane.id > 0))) * std::abs(t) + this->get_lateral_shape(s, t);
        dh_dt = this->get_lateral_shape_derivative(s, t);
    }

    if (lane.s_to_height_offset.size() > 0)
    {
        const std::map<double, HeightOffset>& height_offs = lane.s_to_height_offset;

        auto s0_height_offs_iter = height_offs.upper_bound(s);
        if (s0_height_offs_iter != height_offs.begin())
            s0_height_offs_iter--;

        const double t_outer_brdr = lane.outer_border.evaluate(s);
        const double inner_height = s0_height_offs_iter->second.inner;
        const double outer_height = s0_height_offs_iter->second.outer;
        const double p_t = (t_outer_brdr != t_inner_brdr) ? (t - t_inner_brdr) / (t_outer_brdr - t_inner_brdr) : 0.0;
        h_t += p_t * (outer_height - inner_height) + inner_height;

        if (std::next(s0_height_offs_iter) != height_offs.end())
        {
            /* if successive lane height entry available linearly interpolate */
            const double ds = std::next(s0_height_offs_iter)->first - s0_height_offs_iter->first;
            const double d_lh_inner = std::next(s0_height_offs_iter)->second.inner - inner_height;
            const double dh_inner = (d_lh_inner / ds) * (s - s0_height_offs_iter->first);
            const double d_lh_outer = std::next(s0_height_offs_iter)->second.outer - outer_height;
            const double dh_outer = (d_lh_outer / ds) * (s - s0_height_offs_iter->first);

            h_t += p_t * (dh_outer - dh_inner) + dh_inner;
        }
    }

    Vec3D       e_t{0, 0, 0};
    Vec3D       e_h{0, 0, 0};
    const Vec3D xyz = this->get_xyz(s, t, h_t, nullptr, &e_t, &e_h);
    if (vn)
    {
        // tilt the normal by the lateral slope dh/dt (lateral shape, or cancelled superelevation for level lanes), the surface
        // tangent along t is e_t + dh/dt * e_h
        *vn = (dh_dt == 0.0) ? e_h : normalize(sub(e_h, mut(dh_dt, e_t)));
    }

    return xyz;
}

// OpenDRIVE 1.6.1, 8.4.3 Shape definition: "Between shape profiles, at specific s-coordinates, the road shape is interpolated
// linearly." and "The default value for shape is zero."
static double interpolate_lateral_shape(const std::map<double, CubicProfile>& s_to_lateral_shape,
                                        const double                          s,
                                        const double                          t,
                                        double (CubicProfile::*eval_fn)(const double, const double, const bool) const)
{
    auto s1_shape_iter = s_to_lateral_shape.upper_bound(s);
    if (s1_shape_iter == s_to_lateral_shape.begin())
        return 0.0;

    const auto   s0_shape_iter = std::prev(s1_shape_iter);
    const double h0 = (s0_shape_iter->second.*eval_fn)(t, 0.0, true);
    if (s1_shape_iter == s_to_lateral_shape.end())
        return h0;

    const double h1 = (s1_shape_iter->second.*eval_fn)(t, 0.0, true);
    const double ds = s1_shape_iter->first - s0_shape_iter->first;
    return h0 + (h1 - h0) * ((s - s0_shape_iter->first) / ds);
}

double Road::get_lateral_shape(const double s, const double t) const
{
    return interpolate_lateral_shape(this->s_to_lateral_shape, s, t, &CubicProfile::evaluate);
}

double Road::get_lateral_shape_derivative(const double s, const double t) const
{
    return interpolate_lateral_shape(this->s_to_lateral_shape, s, t, &CubicProfile::derivative);
}

// t values in [t_start, t_end] for which a linear interpolation of the lateral shape at s stays within eps, includes t_start and t_end
std::set<double> Road::approximate_lateral_shape_linear(const double s, const double t_start, const double t_end, const double eps) const
{
    std::set<double> t_vals{t_start, t_end};
    if (t_start >= t_end)
        return t_vals;

    auto s1_shape_iter = this->s_to_lateral_shape.upper_bound(s);
    if (s1_shape_iter == this->s_to_lateral_shape.begin())
        return t_vals; // no shape before the first profile

    // the shape at s is interpolated between the profiles at s0 and s1, sample the polynomial borders of both
    std::vector<const CubicProfile*> shape_profiles{&std::prev(s1_shape_iter)->second};
    if (s1_shape_iter != this->s_to_lateral_shape.end())
        shape_profiles.push_back(&s1_shape_iter->second);

    for (const CubicProfile* shape_profile : shape_profiles)
    {
        if (shape_profile->segments.empty())
            continue;

        // the first polynomial also applies before its t (extend_start), which CubicProfile::approximate_linear does not cover
        const double     t_first_poly = shape_profile->segments.begin()->first;
        std::set<double> t_vals_shape;
        if (t_start < t_first_poly)
            t_vals_shape = shape_profile->segments.begin()->second.approximate_linear(eps, t_start, std::min(t_first_poly, t_end));
        if (t_end > t_first_poly)
        {
            const std::set<double> t_vals_polys = shape_profile->approximate_linear(eps, std::max(t_first_poly, t_start), t_end);
            t_vals_shape.insert(t_vals_polys.begin(), t_vals_polys.end());
        }

        for (const double t : t_vals_shape)
            if (t > t_start && t < t_end)
                t_vals.insert(t);
    }

    return t_vals;
}

std::set<double>
Road::approximate_lane_border_linear(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer) const
{
    std::set<double> s_vals = this->ref_line.approximate_linear(eps, s_start, s_end);

    CubicProfile border = lane.outer_border;
    if (!outer)
    {
        const LaneSection& lanesection = this->s_to_lanesection.at(lane.key.lanesection_s0);
        const Lane&        inner_neighbor_lane = lanesection.get_lane(next_towards_zero(lane.id));
        border = inner_neighbor_lane.outer_border;
    }

    std::set<double> s_vals_brdr = border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_brdr.begin(), s_vals_brdr.end());

    std::set<double> s_vals_lane_height = get_map_keys(lane.s_to_height_offset);
    s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

    std::set<double> s_vals_lateral_shape = get_map_keys(this->s_to_lateral_shape);
    for (const double s_shape : s_vals_lateral_shape)
        if (s_shape > s_start && s_shape < s_end)
            s_vals.insert(s_shape);

    const double     t_max = lane.outer_border.max_value(s_start, s_end);
    std::set<double> s_vals_superelev = this->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
    s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());

    return s_vals;
}

std::set<double> Road::approximate_lane_border_linear(const Lane& lane, const double eps, const bool outer) const
{
    const double s_end = this->get_lanesection_end(lane.key.lanesection_s0);
    return this->approximate_lane_border_linear(lane, lane.key.lanesection_s0, s_end, eps, outer);
}

Line3D Road::get_lane_border_line(const Lane& lane, const double s_start, const double s_end, const double eps, const bool outer) const
{
    std::set<double> s_vals = this->approximate_lane_border_linear(lane, s_start, s_end, eps, outer);

    CubicProfile border = lane.outer_border;
    if (!outer)
    {
        const LaneSection& lanesection = this->s_to_lanesection.at(lane.key.lanesection_s0);
        const Lane&        inner_neighbor_lane = lanesection.get_lane(next_towards_zero(lane.id));
        border = inner_neighbor_lane.outer_border;
    }

    Line3D border_line;
    for (const double& s : s_vals)
    {
        double t = border.evaluate(s);
        if (!outer)
            t = std::nextafter(t, lane.outer_border.evaluate(s)); // ensure t is not on lane boundary but within lane
        border_line.push_back(this->get_surface_pt(s, t));
    }

    return border_line;
}

Line3D Road::get_lane_border_line(const Lane& lane, const double eps, const bool outer) const
{
    const double s_end = this->get_lanesection_end(lane.key.lanesection_s0);
    return this->get_lane_border_line(lane, lane.key.lanesection_s0, s_end, eps, outer);
}

Mesh3D Road::get_lane_mesh(const Lane& lane, const double s_start, const double s_end, const double eps, std::vector<uint32_t>* outline_indices) const
{
    std::set<double> s_vals = this->ref_line.approximate_linear(eps, s_start, s_end);
    std::set<double> s_vals_outer_brdr = lane.outer_border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_outer_brdr.begin(), s_vals_outer_brdr.end());

    const LaneSection& lanesection = this->s_to_lanesection.at(lane.key.lanesection_s0);
    const Lane&        inner_neighbor_lane = lanesection.get_lane(next_towards_zero(lane.id));
    std::set<double>   s_vals_inner_brdr = inner_neighbor_lane.outer_border.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_inner_brdr.begin(), s_vals_inner_brdr.end());
    std::set<double> s_vals_lane_offset = this->lane_offset.approximate_linear(eps, s_start, s_end);
    s_vals.insert(s_vals_lane_offset.begin(), s_vals_lane_offset.end());

    std::set<double> s_vals_lane_height = get_map_keys(lane.s_to_height_offset);
    s_vals.insert(s_vals_lane_height.begin(), s_vals_lane_height.end());

    std::set<double> s_vals_lateral_shape = get_map_keys(this->s_to_lateral_shape);
    for (const double s_shape : s_vals_lateral_shape)
        if (s_shape > s_start && s_shape < s_end)
            s_vals.insert(s_shape);

    const double     t_max = lane.outer_border.max_value(s_start, s_end);
    std::set<double> s_vals_superelev = this->superelevation.approximate_linear(std::atan(eps / std::abs(t_max)), s_start, s_end);
    s_vals.insert(s_vals_superelev.begin(), s_vals_superelev.end());

    /* thin out s_vals array, be removing s vals closer than eps to each other */
    for (auto s_iter = s_vals.begin(); s_iter != s_vals.end();)
    {
        if (std::next(s_iter) != s_vals.end() && std::next(s_iter, 2) != s_vals.end() && ((*std::next(s_iter)) - *s_iter) <= eps)
            s_iter = std::prev(s_vals.erase(std::next(s_iter)));
        else
            s_iter++;
    }

    // OpenDRIVE 1.6.1, 8.4.3: the lateral shape may be curved along t, so each cross section is sampled at the t values needed to
    // approximate the shape within eps (and at t=0, where the crossfall changes its sign), not only at the two lane borders. Level
    // lanes are flat (9.5.8) and keep the two border points.
    const bool subdivide_t = !lane.level && !this->s_to_lateral_shape.empty();

    Mesh3D                   out_mesh;
    std::vector<std::size_t> row_start_indices;
    for (const double& s : s_vals)
    {
        row_start_indices.push_back(out_mesh.vertices.size());

        const double t_outer_brdr = lane.outer_border.evaluate(s);
        const double t_inner_brdr =
            std::nextafter(inner_neighbor_lane.outer_border.evaluate(s), t_outer_brdr); // ensure t is not on lane boundary but within lane

        // sample points of the cross section, ordered from the outer to the inner border
        std::vector<double> t_vals{t_outer_brdr};
        if (subdivide_t)
        {
            const double     t_min = std::min(t_outer_brdr, t_inner_brdr);
            const double     t_max = std::max(t_outer_brdr, t_inner_brdr);
            std::set<double> t_vals_shape = this->approximate_lateral_shape_linear(s, t_min, t_max, eps);
            if (t_min < 0.0 && t_max > 0.0)
                t_vals_shape.insert(0.0);
            for (const double t : t_vals_shape)
                if (t > t_min && t < t_max)
                    t_vals.push_back(t);
            if (t_outer_brdr > t_inner_brdr)
                std::reverse(std::next(t_vals.begin()), t_vals.end()); // the points between the borders are in ascending order
        }
        t_vals.push_back(t_inner_brdr);

        for (const double t : t_vals)
        {
            Vec3D vn{0, 0, 0};
            out_mesh.vertices.push_back(this->get_surface_pt(s, t, &vn));
            out_mesh.normals.push_back(vn);
            out_mesh.st_coordinates.push_back({s, t});
        }
    }
    row_start_indices.push_back(out_mesh.vertices.size());

    // relative position of a vertex within its cross section, 0 at the outer and 1 at the inner border
    const auto get_row_param = [&](const std::size_t row, const std::size_t idx) -> double
    {
        const std::size_t row_start = row_start_indices.at(row);
        const std::size_t row_end = row_start_indices.at(row + 1) - 1;
        const double      t_outer = out_mesh.st_coordinates.at(row_start)[1];
        const double      t_inner = out_mesh.st_coordinates.at(row_end)[1];
        if (t_outer == t_inner)
            return static_cast<double>(idx - row_start) / static_cast<double>(row_end - row_start);
        return (out_mesh.st_coordinates.at(idx)[1] - t_outer) / (t_inner - t_outer);
    };

    // triangulate between consecutive cross sections, which may differ in their number of points, by advancing along the
    // cross section whose next point comes first; for two points per cross section this gives the plain triangle strip
    const bool ccw = lane.id < 0;
    for (std::size_t row = 1; row + 1 < row_start_indices.size(); row++)
    {
        std::size_t       idx_prev = row_start_indices.at(row - 1);
        std::size_t       idx_cur = row_start_indices.at(row);
        const std::size_t idx_prev_end = row_start_indices.at(row) - 1;
        const std::size_t idx_cur_end = row_start_indices.at(row + 1) - 1;
        while (idx_prev < idx_prev_end || idx_cur < idx_cur_end)
        {
            const bool advance_cur =
                (idx_prev == idx_prev_end) || ((idx_cur < idx_cur_end) && (get_row_param(row, idx_cur + 1) <= get_row_param(row - 1, idx_prev + 1)));
            const std::size_t                idx_next = advance_cur ? (idx_cur + 1) : (idx_prev + 1);
            const std::array<std::size_t, 3> indices_tri =
                ccw ? std::array<std::size_t, 3>{idx_prev, idx_cur, idx_next} : std::array<std::size_t, 3>{idx_prev, idx_next, idx_cur};
            out_mesh.indices.insert(out_mesh.indices.end(), indices_tri.begin(), indices_tri.end());
            (advance_cur ? idx_cur : idx_prev) = idx_next;
        }
    }

    if (outline_indices && row_start_indices.size() > 1)
    {
        // outer border, inner border, then the first and the last cross section
        outline_indices->clear();
        const std::size_t num_rows = row_start_indices.size() - 1;
        for (std::size_t row = 0; row + 1 < num_rows; row++)
        {
            outline_indices->push_back(row_start_indices.at(row));
            outline_indices->push_back(row_start_indices.at(row + 1));
        }
        for (std::size_t row = 0; row + 1 < num_rows; row++)
        {
            outline_indices->push_back(row_start_indices.at(row + 1) - 1);
            outline_indices->push_back(row_start_indices.at(row + 2) - 1);
        }
        for (const std::size_t row : {std::size_t(0), num_rows - 1})
        {
            for (std::size_t idx = row_start_indices.at(row); idx + 1 < row_start_indices.at(row + 1); idx++)
            {
                outline_indices->push_back(idx);
                outline_indices->push_back(idx + 1);
            }
        }
    }

    return out_mesh;
}

Mesh3D Road::get_lane_mesh(const Lane& lane, const double eps, std::vector<uint32_t>* outline_indices) const
{
    const double s_end = this->get_lanesection_end(lane.key.lanesection_s0);
    return this->get_lane_mesh(lane, lane.key.lanesection_s0, s_end, eps, outline_indices);
}

Mesh3D Road::get_roadmark_mesh(const Lane& lane, const RoadMark& roadmark, const double eps) const
{
    std::set<double>       s_vals = this->approximate_lane_border_linear(lane, roadmark.s_start, roadmark.s_end, eps, true);
    const std::set<double> s_vals_sway = roadmark.sway.approximate_linear(eps, roadmark.s_start, roadmark.s_end);
    s_vals.insert(s_vals_sway.begin(), s_vals_sway.end());

    Mesh3D out_mesh;
    for (const double& s : s_vals)
    {
        Vec3D        vn_edge_a{0, 0, 0};
        const double t_sway = roadmark.sway.evaluate(s, 0.0, false);
        const double t_edge_a = lane.outer_border.evaluate(s) + roadmark.width * 0.5 + roadmark.t_offset + t_sway;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_a, &vn_edge_a));
        out_mesh.normals.push_back(vn_edge_a);

        Vec3D        vn_edge_b{0, 0, 0};
        const double t_edge_b = t_edge_a - roadmark.width;
        out_mesh.vertices.push_back(this->get_surface_pt(s, t_edge_b, &vn_edge_b));
        out_mesh.normals.push_back(vn_edge_b);
    }

    const std::size_t num_pts = out_mesh.vertices.size();
    for (std::size_t idx = 3; idx < num_pts; idx += 2)
    {
        std::array<size_t, 6> indicies_patch = {idx - 3, idx, idx - 1, idx - 3, idx - 2, idx};
        out_mesh.indices.insert(out_mesh.indices.end(), indicies_patch.begin(), indicies_patch.end());
    }

    return out_mesh;
}

Mesh3D Road::get_road_signal_mesh(const RoadSignal& road_signal) const
{
    // OpenDRIVE 1.6.1, 12.4: the mesh is built at the physical position if one is given, the logical position otherwise
    if (road_signal.position_inertial)
    {
        const RoadSignalPositionInertial& pos = *road_signal.position_inertial;
        const Mat3D                       rot_mat = EulerAnglesToMatrix<double>(pos.roll, pos.pitch, pos.hdg);
        Mesh3D                            road_signal_mesh = road_signal.get_box(road_signal.width, 0.2, road_signal.height);
        for (Vec3D& pt_uvz : road_signal_mesh.vertices)
        {
            pt_uvz = add(MatVecMultiplication(rot_mat, pt_uvz), Vec3D{pos.x, pos.y, pos.z});
            road_signal_mesh.st_coordinates.push_back({road_signal.s0, road_signal.t0});
        }
        return road_signal_mesh;
    }

    if (road_signal.position_road && (road_signal.position_road->road_id.empty() || road_signal.position_road->road_id == this->id))
    {
        const RoadSignalPositionRoad& pos = *road_signal.position_road;
        Mesh3D road_signal_mesh = this->get_road_signal_mesh(road_signal, pos.s, pos.t, pos.z_offset, pos.h_offset, pos.pitch, pos.roll);
        for (Vec2D& st : road_signal_mesh.st_coordinates)
            st = {road_signal.s0, road_signal.t0};
        return road_signal_mesh;
    }

    return this->get_road_signal_mesh(
        road_signal, road_signal.s0, road_signal.t0, road_signal.zOffset, road_signal.hOffset, road_signal.pitch, road_signal.roll);
}

Mesh3D Road::get_road_signal_mesh(const RoadSignal& road_signal,
                                  double            s,
                                  const double      t,
                                  const double      z_offset,
                                  const double      h_offset,
                                  const double      pitch,
                                  const double      roll) const
{
    odr::check_and_repair(
        s >= 0, [&]() { s = 0; }, "Road #%s Signal #%s: s < 0 invalid (s=%f), using s=0", this->id.c_str(), road_signal.id.c_str(), s);
    odr::check_and_repair(
        s <= this->length,
        [&]() { s = this->length; },
        "Road #%s Signal #%s: s > length invalid (s=%f, length=%f), using s=length",
        this->id.c_str(),
        road_signal.id.c_str(),
        s,
        this->length);

    // OpenDRIVE 1.6.1, 12: @hOffset is relative to the driving direction the signal faces, which is reversed for "-"
    const double hdg = (road_signal.orientation == "-") ? h_offset + M_PI : h_offset;
    const Mat3D  rot_mat = EulerAnglesToMatrix<double>(roll, pitch, hdg);
    Mesh3D       road_signal_mesh = road_signal.get_box(road_signal.width, 0.2, road_signal.height);
    Vec3D        e_s, e_t, e_h;
    const Vec3D  p0 = this->get_xyz(s, t, z_offset, &e_s, &e_t, &e_h);
    const Mat3D  base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
    for (Vec3D& pt_uvz : road_signal_mesh.vertices)
    {
        pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
        pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
        pt_uvz = add(pt_uvz, p0);

        road_signal_mesh.st_coordinates.push_back({s, t});
    }
    return road_signal_mesh;
}

Mesh3D Road::get_road_object_mesh(const RoadObject& road_object, const double eps) const
{
    std::vector<RoadObjectRepeat> repeats_copy = road_object.repeats; // make copy to keep method const
    if (repeats_copy.empty())                                         // handle single object as 1 object repeat
    {
        RoadObjectRepeat rp(NAN, 0, 1, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN);
        repeats_copy.push_back(rp);
    }

    const Mat3D rot_mat = EulerAnglesToMatrix<double>(road_object.roll, road_object.pitch, road_object.hdg);

    /* helper functions - object repeat's attributes override object's attributes */
    auto get_t_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.t_start) || std::isnan(r.t_end)) ? road_object.t0 : r.t_start + p * (r.t_end - r.t_start); };

    auto get_z_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    {
        return (std::isnan(r.z_offset_start) || std::isnan(r.z_offset_end)) ? road_object.z0
                                                                            : r.z_offset_start + p * (r.z_offset_end - r.z_offset_start);
    };

    auto get_height_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.height_start) || std::isnan(r.height_end)) ? road_object.height : r.height_start + p * (r.height_end - r.height_start); };

    auto get_width_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.width_start) || std::isnan(r.width_end)) ? road_object.width : r.width_start + p * (r.width_end - r.width_start); };

    auto get_length_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.length_start) || std::isnan(r.length_end)) ? road_object.length : r.length_start + p * (r.length_end - r.length_start); };

    auto get_radius_s = [&](const RoadObjectRepeat& r, const double& p) -> double
    { return (std::isnan(r.radius_start) || std::isnan(r.radius_end)) ? road_object.radius : r.radius_start + p * (r.radius_end - r.radius_start); };

    Mesh3D road_obj_mesh;

    /* an outline supersedes the bounding box / cylinder: repeated instances get the outline instead of a generic object, continuous repeats
     * (distance = 0) of outline objects are drawn as the single outline object */
    const bool has_outlines = !road_object.outlines.empty();
    bool       added_original_outlines = false;
    for (const RoadObjectRepeat& repeat : repeats_copy)
    {
        const double s_start = std::isnan(repeat.s0) ? road_object.s0 : repeat.s0;
        const double s_end = std::min(s_start + repeat.length, this->length);

        if (has_outlines && repeat.distance == 0)
        {
            if (!added_original_outlines)
                road_obj_mesh.add_mesh(this->get_road_object_outlines_mesh(road_object, road_object.s0, road_object.t0, road_object.z0));
            added_original_outlines = true;
        }
        else if (repeat.distance != 0)
        {
            for (double s = s_start; s <= s_end; s += repeat.distance)
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = get_t_s(repeat, p);
                const double z_s = get_z_s(repeat, p);
                if (has_outlines)
                {
                    road_obj_mesh.add_mesh(this->get_road_object_outlines_mesh(road_object, s, t_s, z_s));
                    continue;
                }

                const double height_s = get_height_s(repeat, p);
                const double w_s = get_width_s(repeat, p);
                const double l_s = get_length_s(repeat, p);
                const double r_s = get_radius_s(repeat, p);

                Mesh3D single_road_obj_mesh;
                if (r_s > 0)
                {
                    single_road_obj_mesh = road_object.get_cylinder(eps, r_s, height_s);
                }
                else if (w_s > 0 && l_s > 0)
                {
                    single_road_obj_mesh = road_object.get_box(w_s, l_s, height_s);
                }

                Vec3D       e_s, e_t, e_h;
                const Vec3D p0 = this->get_xyz(s, t_s, z_s, &e_s, &e_t, &e_h);
                const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};
                for (Vec3D& pt_uvz : single_road_obj_mesh.vertices)
                {
                    pt_uvz = MatVecMultiplication(rot_mat, pt_uvz);
                    pt_uvz = MatVecMultiplication(base_mat, pt_uvz);
                    pt_uvz = add(pt_uvz, p0);

                    single_road_obj_mesh.st_coordinates.push_back({s, t_s});
                }

                road_obj_mesh.add_mesh(single_road_obj_mesh);
            }
        }
        else
        {
            Mesh3D continuous_road_obj_mesh;

            const std::array<size_t, 24> idx_patch_template = {1, 5, 4, 1, 4, 0, 2, 7, 6, 2, 3, 7, 1, 6, 5, 1, 2, 6, 0, 4, 7, 0, 7, 3};
            for (const double& s : this->ref_line.approximate_linear(eps, s_start, s_end))
            {
                const double p = (s_end == s_start) ? 1.0 : (s - s_start) / (s_end - s_start);
                const double t_s = get_t_s(repeat, p);
                const double z_s = get_z_s(repeat, p);
                const double height_s = get_height_s(repeat, p);
                const double r_s = get_radius_s(repeat, p);
                /* continuous circular objects (e.g. pipes) are approximated by their diameter */
                const double w_obj = get_width_s(repeat, p);
                const double w_s = (w_obj > 0 || r_s <= 0) ? w_obj : 2 * r_s;

                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s - 0.5 * w_s, z_s));
                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s + 0.5 * w_s, z_s));
                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s + 0.5 * w_s, z_s + height_s));
                continuous_road_obj_mesh.vertices.push_back(this->get_xyz(s, t_s - 0.5 * w_s, z_s + height_s));

                const std::array<Vec2D, 4> s_t_coords = {{{s, t_s - 0.5 * w_s}, {s, t_s + 0.5 * w_s}, {s, t_s + 0.5 * w_s}, {s, t_s - 0.5 * w_s}}};
                continuous_road_obj_mesh.st_coordinates.insert(continuous_road_obj_mesh.st_coordinates.end(), s_t_coords.begin(), s_t_coords.end());

                if (continuous_road_obj_mesh.vertices.size() == 4)
                {
                    const std::array<size_t, 6> front_idx_patch = {0, 2, 1, 0, 3, 2};
                    continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), front_idx_patch.begin(), front_idx_patch.end());
                }

                if (continuous_road_obj_mesh.vertices.size() > 7)
                {
                    const std::size_t      cur_offs = continuous_road_obj_mesh.vertices.size() - 8;
                    std::array<size_t, 24> wall_idx_patch;
                    for (std::size_t idx = 0; idx < idx_patch_template.size(); idx++)
                        wall_idx_patch.at(idx) = idx_patch_template.at(idx) + cur_offs;
                    continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
                }
            }

            const std::size_t           last_idx = continuous_road_obj_mesh.vertices.size() - 1;
            const std::array<size_t, 6> back_idx_patch = {last_idx - 3, last_idx - 2, last_idx - 1, last_idx - 3, last_idx - 1, last_idx};
            continuous_road_obj_mesh.indices.insert(continuous_road_obj_mesh.indices.end(), back_idx_patch.begin(), back_idx_patch.end());

            road_obj_mesh.add_mesh(continuous_road_obj_mesh);
        }
    }

    road_obj_mesh.add_mesh(this->get_road_object_markings_mesh(road_object));
    road_obj_mesh.add_mesh(this->get_road_object_borders_mesh(road_object));

    return road_obj_mesh;
}

Vec3D Road::get_road_object_corner_xyz(const RoadObject&       road_object,
                                       const RoadObjectCorner& corner,
                                       const double            height,
                                       const double            s_ins,
                                       const double            t_ins,
                                       const double            z_ins) const
{
    /* corner position of an object instance; road corners are shifted by the offset of the instance to the original object */
    if (corner.type == RoadObjectCorner::Type::Local_AbsZ || corner.type == RoadObjectCorner::Type::Local_RelZ)
    {
        const Mat3D rot_mat = EulerAnglesToMatrix<double>(road_object.roll, road_object.pitch, road_object.hdg);

        Vec3D       e_s, e_t, e_h;
        const Vec3D p0 = this->get_xyz(s_ins, t_ins, z_ins, &e_s, &e_t, &e_h);
        const Mat3D base_mat{{{e_s[0], e_t[0], e_h[0]}, {e_s[1], e_t[1], e_h[1]}, {e_s[2], e_t[2], e_h[2]}}};

        Vec3D pt = {corner.pt[0], corner.pt[1], corner.pt[2]};
        if (corner.type == RoadObjectCorner::Type::Local_AbsZ)
            pt[2] -= p0[2]; // make road relative
        pt = add(pt, Vec3D{0, 0, height});
        return add(MatVecMultiplication(base_mat, MatVecMultiplication(rot_mat, pt)), p0);
    }
    return this->get_xyz(
        corner.pt[0] + s_ins - road_object.s0, corner.pt[1] + t_ins - road_object.t0, corner.pt[2] + z_ins - road_object.z0 + height);
}

Mesh3D Road::get_road_object_outlines_mesh(const RoadObject& road_object, const double s_ins, const double t_ins, const double z_ins) const
{
    auto get_corner_xyz = [&](const RoadObjectCorner& corner, const double height) -> Vec3D
    { return this->get_road_object_corner_xyz(road_object, corner, height, s_ins, t_ins, z_ins); };

    auto is_point_in_polygon = [](const Vec3D& pt, const std::vector<Vec3D>& polygon) -> bool
    {
        bool inside = false;
        for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
            if (((polygon[i][1] > pt[1]) != (polygon[j][1] > pt[1])) &&
                (pt[0] < (polygon[j][0] - polygon[i][0]) * (pt[1] - polygon[i][1]) / (polygon[j][1] - polygon[i][1]) + polygon[i][0]))
                inside = !inside;
        }
        return inside;
    };

    /* OpenDRIVE 1.6.1 UC_2Lane-RoundAbout-3Arms: closed inner outlines (outer=false) are the physical parts (e.g. concrete noses and a lowered
     * sidewalk) of the closed outer outline containing them, which is only the envelope of the object - such outer outlines are not drawn */
    std::vector<Vec3D> inner_outline_centers;
    for (const RoadObjectOutline& road_object_outline : road_object.outlines)
    {
        if (road_object_outline.outline.size() < 2 || !road_object_outline.closed || road_object_outline.outer)
            continue;
        Vec3D center = {0, 0, 0};
        for (const RoadObjectCorner& corner : road_object_outline.outline)
            center = add(center, get_corner_xyz(corner, 0));
        inner_outline_centers.push_back(mut(1.0 / road_object_outline.outline.size(), center));
    }

    Mesh3D outlines_mesh;
    for (const RoadObjectOutline& road_object_outline : road_object.outlines)
    {
        /* can't add point object */
        if (road_object_outline.outline.size() < 2)
            continue;

        if (road_object_outline.closed && road_object_outline.outer && road_object_outline.outline.size() > 2)
        {
            std::vector<Vec3D> base_points;
            for (const RoadObjectCorner& corner : road_object_outline.outline)
                base_points.push_back(get_corner_xyz(corner, 0));
            const bool is_envelope = std::any_of(inner_outline_centers.begin(),
                                                 inner_outline_centers.end(),
                                                 [&](const Vec3D& center) { return is_point_in_polygon(center, base_points); });
            if (is_envelope)
                continue;
        }

        const bool is_flat_object = std::all_of(
            road_object_outline.outline.begin(), road_object_outline.outline.end(), [](const RoadObjectCorner& c) { return c.height == 0; });

        /* an open outline is a linear feature, it has no area - without height there is nothing to draw */
        if (!road_object_outline.closed && is_flat_object)
            continue;

        Mesh3D outline_road_obj_mesh;

        /* add top outline first - ensure the top vertices are at the front */
        if (!is_flat_object)
        {
            for (const RoadObjectCorner& corner : road_object_outline.outline)
            {
                outline_road_obj_mesh.vertices.push_back(get_corner_xyz(corner, corner.height));
                outline_road_obj_mesh.st_coordinates.push_back({s_ins, t_ins});
            }
        }

        /* add bottom outline */
        for (const RoadObjectCorner& corner : road_object_outline.outline)
        {
            outline_road_obj_mesh.vertices.push_back(get_corner_xyz(corner, 0));
            outline_road_obj_mesh.st_coordinates.push_back({s_ins, t_ins});
        }

        /* run 2D triangulation on top vertices, closed outlines only */
        if (road_object_outline.closed)
        {
            const std::vector<size_t> idx_patch_top =
                mapbox::earcut<size_t>(outline_road_obj_mesh.vertices.data(), road_object_outline.outline.size());
            outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), idx_patch_top.begin(), idx_patch_top.end());
        }

        /* add walls, open outlines have no closing wall */
        if (!is_flat_object)
        {
            const std::size_t N = road_object_outline.outline.size();
            for (std::size_t idx = 0; idx < N - 1; idx++)
            {
                std::array<size_t, 6> wall_idx_patch = {idx, idx + N, idx + 1, idx + 1, idx + N, idx + N + 1};
                outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), wall_idx_patch.begin(), wall_idx_patch.end());
            }

            if (road_object_outline.closed)
            {
                std::array<size_t, 6> last_idx_patch = {N - 1, 2 * N - 1, 0, 0, 2 * N - 1, N};
                outline_road_obj_mesh.indices.insert(outline_road_obj_mesh.indices.end(), last_idx_patch.begin(), last_idx_patch.end());
            }
        }

        outlines_mesh.add_mesh(outline_road_obj_mesh);
    }

    return outlines_mesh;
}

/* flat strip of the given width centered on the polyline, trimmed by start/stop offset and split into dashes of line_length separated by
 * space_length (solid if space_length is 0) */
static Mesh3D get_polyline_strip_mesh(const std::vector<Vec3D>& polyline,
                                      const double              width,
                                      const double              line_length,
                                      const double              space_length,
                                      const double              start_offset,
                                      const double              stop_offset)
{
    Mesh3D strip_mesh;
    if (polyline.size() < 2 || width <= 0)
        return strip_mesh;

    std::vector<double> arc_lengths = {0};
    for (std::size_t idx = 1; idx < polyline.size(); idx++)
        arc_lengths.push_back(arc_lengths.back() + euclDistance(polyline.at(idx - 1), polyline.at(idx)));

    /* lateral direction in the xy plane of the segment starting at seg_idx */
    auto get_seg_normal = [&](const std::size_t seg_idx) -> Vec3D
    {
        const Vec3D dir = sub(polyline.at(seg_idx + 1), polyline.at(seg_idx));
        const Vec3D n = {-dir[1], dir[0], 0};
        return (n[0] == 0 && n[1] == 0) ? n : normalize(n);
    };

    auto add_cross_section = [&](const Vec3D& pt, const Vec3D& normal)
    {
        strip_mesh.vertices.push_back(add(pt, mut(0.5 * width, normal)));
        strip_mesh.vertices.push_back(sub(pt, mut(0.5 * width, normal)));
    };

    const double total_length = arc_lengths.back();
    const double begin = std::max(0.0, start_offset);
    const double end = std::min(total_length, total_length - stop_offset);
    const bool   is_solid = space_length <= 0 || line_length <= 0;
    for (double dash_start = begin; dash_start < end; dash_start += is_solid ? end : (line_length + space_length))
    {
        const double dash_end = is_solid ? end : std::min(dash_start + line_length, end);
        if (dash_end - dash_start <= 0)
            continue;

        const std::size_t first_vert_idx = strip_mesh.vertices.size();
        std::size_t       seg_idx = 0;
        while (seg_idx + 2 < polyline.size() && arc_lengths.at(seg_idx + 1) <= dash_start)
            seg_idx++;

        auto get_pt = [&](const double a, const std::size_t i) -> Vec3D
        {
            const double seg_len = arc_lengths.at(i + 1) - arc_lengths.at(i);
            const double p = (seg_len == 0) ? 0 : (a - arc_lengths.at(i)) / seg_len;
            return add(polyline.at(i), mut(p, sub(polyline.at(i + 1), polyline.at(i))));
        };

        add_cross_section(get_pt(dash_start, seg_idx), get_seg_normal(seg_idx));
        while (seg_idx + 2 < polyline.size() && arc_lengths.at(seg_idx + 1) < dash_end)
        {
            /* polyline vertex inside the dash - use the averaged normal of both segments */
            const Vec3D n_sum = add(get_seg_normal(seg_idx), get_seg_normal(seg_idx + 1));
            add_cross_section(polyline.at(seg_idx + 1), (n_sum[0] == 0 && n_sum[1] == 0) ? n_sum : normalize(n_sum));
            seg_idx++;
        }
        add_cross_section(get_pt(dash_end, seg_idx), get_seg_normal(seg_idx));

        for (std::size_t idx = first_vert_idx; idx + 3 < strip_mesh.vertices.size(); idx += 2)
        {
            const std::array<size_t, 6> idx_patch = {idx, idx + 1, idx + 2, idx + 1, idx + 3, idx + 2};
            strip_mesh.indices.insert(strip_mesh.indices.end(), idx_patch.begin(), idx_patch.end());
        }
    }

    return strip_mesh;
}

/* find the corner with the given id in the outlines of the road object */
static const RoadObjectCorner* find_road_object_corner(const RoadObject& road_object, const int corner_id)
{
    for (const RoadObjectOutline& outline : road_object.outlines)
    {
        for (const RoadObjectCorner& corner : outline.outline)
        {
            if (corner.id == corner_id)
                return &corner;
        }
    }
    return nullptr;
}

Mesh3D Road::get_road_object_markings_mesh(const RoadObject& road_object) const
{
    Mesh3D markings_mesh;
    for (const RoadObjectMarking& marking : road_object.markings)
    {
        std::vector<Vec3D> polyline;
        if (!marking.corner_references.empty())
        {
            /* straight lines between the referenced outline points, lifted by the marking thickness */
            for (const int corner_id : marking.corner_references)
            {
                const RoadObjectCorner* corner = find_road_object_corner(road_object, corner_id);
                if (corner)
                    polyline.push_back(
                        this->get_road_object_corner_xyz(road_object, *corner, marking.z_offset, road_object.s0, road_object.t0, road_object.z0));
            }
        }
        else
        {
            /* side of the bounding box in the local u/v system; left/right run along +u, front/rear along +v */
            const double                                      l = road_object.length;
            const double                                      w = road_object.width;
            const std::map<std::string, std::array<Vec3D, 2>> side_to_pts = {{"left", {Vec3D{-l / 2, w / 2, 0}, Vec3D{l / 2, w / 2, 0}}},
                                                                             {"right", {Vec3D{-l / 2, -w / 2, 0}, Vec3D{l / 2, -w / 2, 0}}},
                                                                             {"front", {Vec3D{l / 2, -w / 2, 0}, Vec3D{l / 2, w / 2, 0}}},
                                                                             {"rear", {Vec3D{-l / 2, -w / 2, 0}, Vec3D{-l / 2, w / 2, 0}}}};
            if (side_to_pts.find(marking.side) == side_to_pts.end())
                continue;
            for (const Vec3D& pt_uv : side_to_pts.at(marking.side))
            {
                const RoadObjectCorner corner(-1, pt_uv, 0, RoadObjectCorner::Type::Local_RelZ);
                polyline.push_back(
                    this->get_road_object_corner_xyz(road_object, corner, marking.z_offset, road_object.s0, road_object.t0, road_object.z0));
            }
        }

        markings_mesh.add_mesh(
            get_polyline_strip_mesh(polyline, marking.width, marking.line_length, marking.space_length, marking.start_offset, marking.stop_offset));
    }

    for (std::size_t idx = markings_mesh.st_coordinates.size(); idx < markings_mesh.vertices.size(); idx++)
        markings_mesh.st_coordinates.push_back({road_object.s0, road_object.t0});

    return markings_mesh;
}

Mesh3D Road::get_road_object_borders_mesh(const RoadObject& road_object) const
{
    Mesh3D borders_mesh;
    for (const RoadObjectBorder& border : road_object.borders)
    {
        const auto outline_iter = std::find_if(road_object.outlines.begin(),
                                               road_object.outlines.end(),
                                               [&](const RoadObjectOutline& outline) { return outline.id == border.outline_id; });
        if (outline_iter == road_object.outlines.end())
            continue;

        /* the border follows the top of the outline (the road surface for flat outlines) */
        std::vector<Vec3D> polyline;
        if (border.use_complete_outline)
        {
            for (const RoadObjectCorner& corner : outline_iter->outline)
                polyline.push_back(
                    this->get_road_object_corner_xyz(road_object, corner, corner.height, road_object.s0, road_object.t0, road_object.z0));
            if (outline_iter->closed && !polyline.empty())
                polyline.push_back(polyline.front());
        }
        else
        {
            for (const int corner_id : border.corner_references)
            {
                const auto corner_iter = std::find_if(outline_iter->outline.begin(),
                                                      outline_iter->outline.end(),
                                                      [&](const RoadObjectCorner& corner) { return corner.id == corner_id; });
                if (corner_iter != outline_iter->outline.end())
                    polyline.push_back(this->get_road_object_corner_xyz(
                        road_object, *corner_iter, corner_iter->height, road_object.s0, road_object.t0, road_object.z0));
            }
        }

        borders_mesh.add_mesh(get_polyline_strip_mesh(polyline, border.width, 0, 0, 0, 0));
    }

    for (std::size_t idx = borders_mesh.st_coordinates.size(); idx < borders_mesh.vertices.size(); idx++)
        borders_mesh.st_coordinates.push_back({road_object.s0, road_object.t0});

    return borders_mesh;
}

} // namespace odr
