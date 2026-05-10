#include "mission_manager.hpp"
MissionManager::MissionManager() {}
void MissionManager::add_waypoint(double lat, double lon, double alt) { waypoints_.push_back({lat, lon, alt}); }
Waypoint MissionManager::get_current_target() const { return (current_wp_index_ < waypoints_.size()) ? waypoints_[current_wp_index_] : Waypoint{0,0,0}; }
bool MissionManager::next_waypoint() { if (current_wp_index_ < waypoints_.size()) { current_wp_index_++; return true; } return false; }
bool MissionManager::is_mission_finished() const { return current_wp_index_ >= waypoints_.size(); }
bool MissionManager::is_at_destination(double cur_lat, double cur_lon, double tolerance_m) { return !is_mission_finished() && calculate_distance(cur_lat, cur_lon, get_current_target().lat, get_current_target().lon) < tolerance_m; }
double MissionManager::calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * M_PI / 180.0, dlon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dlat/2)*std::sin(dlat/2) + std::cos(lat1*M_PI/180.0)*std::cos(lat2*M_PI/180.0)*std::sin(dlon/2)*std::sin(dlon/2);
    return 6371000.0 * 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
}
double MissionManager::calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double dlon = (lon2 - lon1) * M_PI / 180.0, l1 = lat1 * M_PI / 180.0, l2 = lat2 * M_PI / 180.0;
    return std::atan2(std::sin(dlon)*std::cos(l2), std::cos(l1)*std::sin(l2)-std::sin(l1)*std::cos(l2)*std::cos(dlon));
}
void MissionManager::reset() { current_wp_index_ = 0; }
