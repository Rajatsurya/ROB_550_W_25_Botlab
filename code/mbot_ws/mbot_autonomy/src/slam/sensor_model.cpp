#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.075),
	occupancy_threshold_(10),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3)
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    bfs_offsets_.clear();
    
    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            bfs_offsets_.emplace_back(dx, dy);
        }
    }

    std::sort(bfs_offsets_.begin(), bfs_offsets_.end(), [](const Point<int>& a, const Point<int>& b) {
        return (a.x * a.x + a.y * a.y) < (b.x * b.x + b.y * b.y);
    });

    max_offset_norm = std::sqrt(2) * search_range * search_range;
    // max_scan_score = 0.0;
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    /// TODO: Compute the likelihood of the given particle using the provided laser scan and map. 
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    double scanScore = 0.0;

    // for(auto& ray : movingScan){
    //     Point<float> endPoint = ray.origin + Point<float>(ray.range * cos(ray.theta), ray.range * sin(ray.theta));
    //     auto rayEnd = global_position_to_grid_cell(endPoint, map);

    //     if(map.logOdds(rayEnd.x, rayEnd.y) > 0.0){
    //         scanScore += 1.0;
    //     }
    // }

    for(auto& ray : movingScan){
        scanScore += scoreRay(ray, map);
    }

    return scanScore; // Placeholder
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get the end point of the ray on the map
    auto rayEnd = getRayEndPointOnMap(ray, map); 

    if (!map.isCellInGrid(rayEnd.x, rayEnd.y)) {
        return 0.0; // Return 0 if the ray endpoint is outside the grid
    }

    auto nearest_occupiedCell = gridBFS(rayEnd, map); // Find the nearest occupied cell
    if (nearest_occupiedCell != rayEnd) {
        double offset_distance = (rayEnd - nearest_occupiedCell).norm();
        return NormalPdf(offset_distance / max_offset_norm);
    }

    return 0.0; // Return 0 if no occupied cell is found
}

double SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    // Explore neighbors
    for (const auto& offset : bfs_offsets_) {
        Point<int> neighbor(end_point.x + offset.x, end_point.y + offset.y);

        if (map.isCellInGrid(neighbor.x, neighbor.y) &&
            map.isCellOccupied(neighbor.x, neighbor.y))
        {
            return neighbor;            // Found an occupied cell
        }
    }
    // If no occupied cell is found, return the original point
    return end_point; // Return the original point if no occupied cell is found
}

Point<int> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)    // yui, change to int
// Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    Point<float> endPoint = ray.origin + Point<float>(ray.range * cos(ray.theta), ray.range * sin(ray.theta));
    auto rayEnd = global_position_to_grid_cell(endPoint, map);

    return rayEnd;
}
