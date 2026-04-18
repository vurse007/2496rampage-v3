#pragma once
#include <vector>
#include <cmath>
#include <algorithm>


namespace lynx {

struct Waypoint {
    double x; // inches
    double y; // inches
    double heading; // degrees, 0 = North
    double velocity; // 0-127 (use inches/sec for true seconds)
    double curvature = 0.0; // 1/inches, signed. omega = v * k
    double time = 0.0; // seconds from start of path

    Waypoint(double x = 0, double y = 0, double heading = 0.0, double velocity = 100.0, double curvature = 0.0, double time = 0.0)
        : x(x), y(y), heading(heading), velocity(velocity), curvature(curvature), time(time) {}

    double distance_to(const Waypoint& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return std::sqrt(dx * dx + dy * dy);
    }
};

class BezierPath {
public:

    struct CubicSegment {
        double x0, y0, x1, y1, x2, y2, x3, y3;
        double start_velocity = 100.0;
        double end_velocity = 100.0;

        CubicSegment(double x0, double y0,
                     double x1, double y1,
                     double x2, double y2,
                     double x3, double y3,
                     double sv = 100.0, double ev = 100.0)
            : x0(x0), y0(y0), x1(x1), y1(y1),
              x2(x2), y2(y2), x3(x3), y3(y3),
              start_velocity(sv), end_velocity(ev) {}
    };

    struct ControlPoint {
        double x, y;
        double heading; // IMU degrees: 0=North, 90=East
        double velocity;

        ControlPoint(double x, double y, double heading = 0.0, double velocity = 100.0)
            : x(x), y(y), heading(heading), velocity(velocity) {}
    };

private:

    static double bezier_x(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return mt*mt*mt*s.x0 + 3*mt*mt*t*s.x1 + 3*mt*t*t*s.x2 + t*t*t*s.x3;
    }
    static double bezier_y(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return mt*mt*mt*s.y0 + 3*mt*mt*t*s.y1 + 3*mt*t*t*s.y2 + t*t*t*s.y3;
    }
    static double bezier_dx(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return 3*mt*mt*(s.x1 - s.x0) + 6*mt*t*(s.x2 - s.x1) + 3*t*t*(s.x3 - s.x2);
    }
    static double bezier_dy(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return 3*mt*mt*(s.y1 - s.y0) + 6*mt*t*(s.y2 - s.y1) + 3*t*t*(s.y3 - s.y2);
    }
    static double bezier_ddx(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return 6*mt*(s.x2 - 2*s.x1 + s.x0) + 6*t*(s.x3 - 2*s.x2 + s.x1);
    }
    static double bezier_ddy(const CubicSegment& s, double t) {
        double mt = 1.0 - t;
        return 6*mt*(s.y2 - 2*s.y1 + s.y0) + 6*t*(s.y3 - 2*s.y2 + s.y1);
    }

    static double tangent_to_imu_heading(double dx, double dy) {
        double math_deg = std::atan2(dy, dx) * 180.0 / M_PI;
        return std::fmod(90.0 - math_deg + 360.0, 360.0);
    }

    // signed curvature: positive = left turn
    static double curvature_at(const CubicSegment& s, double t) {
        double dx = bezier_dx(s, t);
        double dy = bezier_dy(s, t);
        double ddx = bezier_ddx(s, t);
        double ddy = bezier_ddy(s, t);
        double denom = std::pow(dx*dx + dy*dy, 1.5);
        if (denom < 1e-6) return 0.0;
        return (dx*ddy - dy*ddx) / denom;
    }

    static double limit_speed_by_curvature(double k, double max_speed, double track_width) {
        if (std::abs(k) < 1e-9) return max_speed;
        return max_speed / (1.0 + std::abs(k) * 0.5 * track_width);
    }

    static std::vector<std::pair<double,double>> build_arc_table(const CubicSegment& seg, int subdivisions = 500) {
        std::vector<std::pair<double,double>> table;
        table.reserve(subdivisions + 1);
        double arc = 0.0;
        double prev_x = bezier_x(seg, 0.0);
        double prev_y = bezier_y(seg, 0.0);
        table.push_back({0.0, 0.0});
        for (int i = 1; i <= subdivisions; i++) {
            double t = (double)i / subdivisions;
            double cx = bezier_x(seg, t);
            double cy = bezier_y(seg, t);
            arc += std::sqrt((cx-prev_x)*(cx-prev_x) + (cy-prev_y)*(cy-prev_y));
            table.push_back({t, arc});
            prev_x = cx; prev_y = cy;
        }
        return table;
    }

    static double arc_to_t(const std::vector<std::pair<double,double>>& table, double target_arc) {
        if (target_arc <= 0.0) return 0.0;
        if (target_arc >= table.back().second) return 1.0;
        int lo = 0, hi = (int)table.size() - 1;
        while (hi - lo > 1) {
            int mid = (lo + hi) / 2;
            if (table[mid].second < target_arc) lo = mid;
            else hi = mid;
        }
        double denom = table[hi].second - table[lo].second;
        double frac = (denom > 1e-9)? (target_arc - table[lo].second) / denom : 0.0;
        return table[lo].first + frac * (table[hi].first - table[lo].first);
    }

public:

    static std::vector<Waypoint> from_segment(const CubicSegment& seg, double spacing_inches = 0.5) {
        auto arc_table = build_arc_table(seg);
        double total_arc = arc_table.back().second;
        if (total_arc < 0.001) return {};
        int num_points = std::max(2, (int)std::ceil(total_arc / spacing_inches) + 1);
        std::vector<Waypoint> waypoints;
        waypoints.reserve(num_points);
        for (int i = 0; i < num_points; i++) {
            double frac = (double)i / (num_points - 1);
            double s = frac * total_arc;
            double t = arc_to_t(arc_table, s);
            double x = bezier_x(seg, t);
            double y = bezier_y(seg, t);
            double heading = tangent_to_imu_heading(bezier_dx(seg, t), bezier_dy(seg, t));
            double velocity = std::max(0.0, seg.start_velocity + frac * (seg.end_velocity - seg.start_velocity));
            waypoints.emplace_back(x, y, heading, velocity, 0.0, 0.0);
        }
        return waypoints;
    }

    static std::vector<Waypoint> from_segment_profiled(const CubicSegment& seg, double spacing_inches = 0.5, double max_speed = 127.0, double max_accel = 150.0, double track_width = 12.0) {
        auto table = build_arc_table(seg, 500);
        double total = table.back().second;
        if (total < 0.001) return {};
    
        int n = std::max(2, (int)std::ceil(total / spacing_inches) + 1);
        std::vector<double> s_vals; s_vals.reserve(n);
        std::vector<double> v_cap; v_cap.reserve(n);
        std::vector<Waypoint> pts; pts.reserve(n);
    
        for (int i = 0; i < n; ++i) {
            double frac = (double)i / (n - 1);
            double s = frac * total;
            double t = arc_to_t(table, s);
            double x = bezier_x(seg, t);
            double y = bezier_y(seg, t);
            double h = tangent_to_imu_heading(bezier_dx(seg, t), bezier_dy(seg, t));
            double k = curvature_at(seg, t);
            double v_k = limit_speed_by_curvature(k, max_speed, track_width);
            double v_user = seg.start_velocity + frac * (seg.end_velocity - seg.start_velocity);
            double v = std::min(v_k, v_user);
            s_vals.push_back(s);
            v_cap.push_back(v);
            pts.emplace_back(x, y, h, v, k, 0.0);
        }
    
        pts[0].velocity = std::min(v_cap[0], seg.start_velocity);
        for (int i = 1; i < n; ++i) {
            double ds = s_vals[i] - s_vals[i-1];
            double v_possible = std::sqrt(pts[i-1].velocity * pts[i-1].velocity + 2.0 * max_accel * ds);
            pts[i].velocity = std::min(v_cap[i], v_possible);
        }
        pts.back().velocity = std::min(pts.back().velocity, seg.end_velocity);
        for (int i = n - 2; i >= 0; --i) {
            double ds = s_vals[i+1] - s_vals[i];
            double v_possible = std::sqrt(pts[i+1].velocity * pts[i+1].velocity + 2.0 * max_accel * ds);
            pts[i].velocity = std::min(pts[i].velocity, v_possible);
        }
        for (auto& w : pts) w.velocity = std::clamp(w.velocity, 0.0, max_speed);
    
        pts[0].time = 0.0;
        for (int i = 1; i < n; ++i) {
            double ds = s_vals[i] - s_vals[i-1];
            double vsum = pts[i-1].velocity + pts[i].velocity;
            double dt = (vsum > 1e-6) ? (2.0 * ds / vsum) : 0.0;
            pts[i].time = pts[i-1].time + dt;
        }
        return pts;
    }

    static std::vector<Waypoint> from_control_points(const std::vector<ControlPoint>& pts, double tangent_scale = 0.4, double spacing_inches = 0.5, double max_speed = 127.0, double max_accel = 150.0, double track_width = 12.0) {
        if (pts.size() < 2) return {};
        std::vector<Waypoint> all_waypoints;
        double time_offset = 0.0;

        for (int i = 0; i < (int)pts.size() - 1; i++) {
            const ControlPoint& p0 = pts[i];
            const ControlPoint& p1 = pts[i + 1];
            double dist = std::hypot(p1.x-p0.x, p1.y-p0.y);
            double handle = std::max(dist * tangent_scale, 0.1);
            double h0_rad = p0.heading * M_PI / 180.0;
            double h1_rad = p1.heading * M_PI / 180.0;

            CubicSegment seg(
                p0.x, p0.y,
                p0.x + handle * std::sin(h0_rad), p0.y + handle * std::cos(h0_rad),
                p1.x - handle * std::sin(h1_rad), p1.y - handle * std::cos(h1_rad),
                p1.x, p1.y,
                p0.velocity, p1.velocity);

            auto seg_wps = from_segment_profiled(seg, spacing_inches, max_speed, max_accel, track_width);
            for (auto& wp : seg_wps) wp.time += time_offset;

            if (!all_waypoints.empty() &&!seg_wps.empty())
                seg_wps.erase(seg_wps.begin());

            all_waypoints.insert(all_waypoints.end(), seg_wps.begin(), seg_wps.end());
            if (!all_waypoints.empty())
                time_offset = all_waypoints.back().time;
        }
        return all_waypoints;
    }

    static std::vector<Waypoint> auto_path(const std::vector<std::pair<double,double>>& xy_points, double velocity = 100.0, double tangent_scale = 0.4, double spacing_inches = 0.5, double max_speed = 127.0, double max_accel = 150.0, double track_width = 12.0) {
        if (xy_points.size() < 2) return {};
        std::vector<ControlPoint> pts;
        pts.reserve(xy_points.size());
        for (int i = 0; i < (int)xy_points.size(); i++) {
            double heading;
            if (i == 0) {
                heading = tangent_to_imu_heading(xy_points[1].first - xy_points[0].first, xy_points[1].second - xy_points[0].second);
            } else if (i == (int)xy_points.size() - 1) {
                heading = tangent_to_imu_heading(xy_points[i].first - xy_points[i-1].first, xy_points[i].second - xy_points[i-1].second);
            } else {
                double dx1 = xy_points[i].first - xy_points[i-1].first;
                double dy1 = xy_points[i].second - xy_points[i-1].second;
                double dx2 = xy_points[i+1].first - xy_points[i].first;
                double dy2 = xy_points[i+1].second - xy_points[i].second;
                double len1 = std::hypot(dx1, dy1);
                double len2 = std::hypot(dx2, dy2);
                if (len1 > 0.001) { dx1 /= len1; dy1 /= len1; }
                if (len2 > 0.001) { dx2 /= len2; dy2 /= len2; }
                heading = tangent_to_imu_heading(dx1 + dx2, dy1 + dy2);
            }
            pts.emplace_back(xy_points[i].first, xy_points[i].second, heading, velocity);
        }
        return from_control_points(pts, tangent_scale, spacing_inches, max_speed, max_accel, track_width);
    }

    static std::vector<Waypoint> make(std::initializer_list<ControlPoint> pts, double tangent_scale = 0.4, double spacing_inches = 0.5, double max_speed = 127.0, double max_accel = 150.0, double track_width = 12.0) {
        return from_control_points(std::vector<ControlPoint>(pts), tangent_scale, spacing_inches, max_speed, max_accel, track_width);
    }

    static Waypoint sample_at_time(const std::vector<Waypoint>& path, double query_time) {
        if (path.empty()) return Waypoint();
        if (query_time <= path.front().time) return path.front();
        if (query_time >= path.back().time) return path.back();
        size_t lo = 0, hi = path.size() - 1;
        while (hi - lo > 1) {
            size_t mid = (lo + hi) / 2;
            if (path[mid].time < query_time) lo = mid;
            else hi = mid;
        }
        const Waypoint& a = path[lo];
        const Waypoint& b = path[hi];
        double dt = b.time - a.time;
        if (dt < 1e-9) return a;
        double frac = (query_time - a.time) / dt;
        double x = a.x + frac * (b.x - a.x);
        double y = a.y + frac * (b.y - a.y);
        double dh = std::fmod(b.heading - a.heading + 540.0, 360.0) - 180.0;
        double h = std::fmod(a.heading + frac * dh + 360.0, 360.0);
        double v = a.velocity + frac * (b.velocity - a.velocity);
        double k = a.curvature + frac * (b.curvature - a.curvature);
        return Waypoint(x, y, h, v, k, query_time);
    }

}; // class BezierPath

inline std::vector<Waypoint> path(std::initializer_list<BezierPath::ControlPoint> pts, double tangent_scale = 0.4, double spacing_inches = 0.5, double max_speed = 127.0, double max_accel = 150.0, double track_width = 12.0) {
    return BezierPath::make(pts, tangent_scale, spacing_inches, max_speed, max_accel, track_width);
}

} // namespace lynx