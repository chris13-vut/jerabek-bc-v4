#include "mission_manager.hpp"

MissionManager::MissionManager() {}

void MissionManager::add_waypoint(double lat, double lon, double alt) {
    waypoints_.push_back({lat, lon, alt});
}

Waypoint MissionManager::get_current_target() const {
    if (current_wp_index_ < waypoints_.size()) return waypoints_[current_wp_index_];
    return {0, 0, 0};
}

bool MissionManager::next_waypoint() {
    if (current_wp_index_ < waypoints_.size()) {
        current_wp_index_++;
        return true;
    }
    return false;
}

bool MissionManager::is_mission_finished() const {
    return current_wp_index_ >= waypoints_.size();
}

bool MissionManager::is_at_destination(double cur_lat, double cur_lon, double tolerance_m) {
    if (is_mission_finished()) return false;
    Waypoint target = get_current_target();
    return calculate_distance(cur_lat, cur_lon, target.lat, target.lon) < tolerance_m;
}

double MissionManager::calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0;
    double dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat/2) * std::sin(dlat/2) +
               std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
               std::sin(dlon/2) * std::sin(dlon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    return 6371000.0 * c; 
}

double MissionManager::calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double dlon_rad = (lon2 - lon1) * M_PI / 180.0;
    
    double y = std::sin(dlon_rad) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) - std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(dlon_rad);
    return std::atan2(y, x); 
}

void MissionManager::reset() {
    current_wp_index_ = 0;
}
