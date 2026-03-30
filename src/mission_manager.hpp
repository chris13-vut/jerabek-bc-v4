#ifndef MISSION_MANAGER_HPP_
#define MISSION_MANAGER_HPP_

#include <vector>
#include <cmath>

struct Waypoint { double lat, lon, alt; };

class MissionManager {
public:
    MissionManager();
    void add_waypoint(double lat, double lon, double alt);
    Waypoint get_current_target() const;
    bool is_at_destination(double cur_lat, double cur_lon, double tolerance_m);
    bool next_waypoint();
    bool is_mission_finished() const;
    double calculate_distance(double lat1, double lon1, double lat2, double lon2);
    double calculate_bearing(double lat1, double lon1, double lat2, double lon2);
    size_t get_current_index() const { return current_wp_index_; }
    size_t get_total_waypoints() const { return waypoints_.size(); }
    void reset(); 

private:
    std::vector<Waypoint> waypoints_;
    size_t current_wp_index_ = 0;
};

#endif
