#include <planning/astar.hpp>
#include <algorithm>
#include <chrono>

using namespace std::chrono;

mbot_lcm_msgs::path2D_t search_for_path(mbot_lcm_msgs::pose2D_t start,
                                             mbot_lcm_msgs::pose2D_t goal,
                                             const ObstacleDistanceGrid& distances,
                                             const SearchParams& params)
{
    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    bool found_path = false;
    
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    Node* startNode = new Node(startCell.x, startCell.y);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    
    mbot_lcm_msgs::path2D_t path;
    path.utime = start.utime;

    if(startNode->cell == goalNode->cell){
        printf("[A*] Start and goal are the same!!\n");
        found_path = true;
        path.path.push_back(start);
        path.path_length = 1;
        return path;
    }

    PriorityQueue openList;
    std::vector<Node*> closedList;

    startNode->g_cost = 0.0;
    startNode->h_cost = h_cost(startNode, goalNode, distances);
    openList.push(startNode);

    while(!openList.empty()){

        Node* currentNode = openList.pop();

        if (currentNode->cell == goalNode->cell){
            found_path = true;
            goalNode = currentNode;
            break;
        }

        std::vector<Node*> children = expand_node(currentNode, distances, params);
        for(auto&& child : children){
            
            if(child->cell == goalNode->cell){
                found_path = true;
                goalNode = child;
                break;
            }

            if(!is_in_list(child, closedList)){     // if child is not in the closed list

                double gcost = currentNode->g_cost + g_cost(currentNode, child, distances, params);
                if(gcost < child->g_cost){
                    child->parent = currentNode;
                    child->g_cost = gcost;
                    child->h_cost = h_cost(child, goalNode, distances);
                }

                if(!openList.is_member(child)){     // if child is not in the open list
                    openList.push(child);
                }
                else{
                    Node* existingChild = openList.get_member(child);
                    existingChild = child;
                }
            }
        }

        if(found_path) break;
        closedList.push_back(currentNode);
    }


    if (found_path)
    {
        printf("[A*] Found a path\n");
        auto nodePath = extract_node_path(goalNode, startNode);
        auto prune_nodePath = prune_node_path(nodePath);
        path.path = extract_pose_path(prune_nodePath, distances);
        // Remove last pose, and add the goal pose
        path.path.pop_back();
        path.path.push_back(goal);
    }

    else printf("[A*] Didn't find a path\n");
    path.path_length = path.path.size();

    return path;
}


// heuristic cost from current node to goal node
double h_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances)
{
    double h_cost = 0.0;
    ////////////////// TODO: Implement your heuristic //////////////////////////
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    
    // Diagonal distance
    h_cost = (dx + dy) + (sqrt(2) - 2) * std::min(dx, dy);
    
    // Manhattan distance
    // h_cost = dx + dy;

    // Euclidean distance
    // h_cost = sqrt(dx*dx + dy*dy);
    
    return h_cost;
}

// cost from current node to parent node
double g_cost(Node* from, Node* goal, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    double g_cost = 0.0;
    ////////////////// TODO: Implement your goal cost, use obstacle distances //////////////////////////
    int dx = abs(from->cell.x - goal->cell.x);
    int dy = abs(from->cell.y - goal->cell.y);
    // Euclidean distance
    g_cost = sqrt(dx*dx + dy*dy);

    // Obstacle distance cost, use the distance to the nearest obstacle
    double cellDistance = distances(goal->cell.x, goal->cell.y);
    double minDistance = params.minDistanceToObstacle * 1.01;       // robotRadius, 1.01 for narrow path
    double maxDistance = params.maxDistanceWithCost;                // 10.0 * minDistance
    // double maxDistance = minDistance * 5;                // 10.0 * minDistance

    if(cellDistance > minDistance && cellDistance < maxDistance){
        g_cost += pow(maxDistance / cellDistance, params.distanceCostExponent);
    }

    return g_cost;
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params)
{
    std::vector<Node*> children;
    ////////////////// TODO: Implement your expand node algorithm //////////////////////////
    const int xDeltas[8] = {1, 0, -1, 0, 1, 1, -1, -1};
    const int yDeltas[8] = {0, 1, 0, -1, 1, -1, 1, -1};

    double minDistance = params.minDistanceToObstacle * 1.01;       // robotRadius, 1.01 for narrow path

    for (int n = 0; n < 8; n++){

        cell_t adjacentCell(node->cell.x + xDeltas[n], node->cell.y + yDeltas[n]);
        double cellDistance = distances(adjacentCell.x, adjacentCell.y);

        if (distances.isCellInGrid(adjacentCell.x, adjacentCell.y) && (cellDistance > minDistance)){
            
            Node* child = new Node(adjacentCell.x, adjacentCell.y);

            child->parent = node;
            children.push_back(child);
        }
    }

    return children;
}

std::vector<Node*> extract_node_path(Node* goal_node, Node* start_node)
{
    std::vector<Node*> path;
    ////////////////// TODO: Implement your extract node function //////////////////////////
    // Traverse nodes and add parent nodes to the vector

    path.push_back(goal_node);
    Node* current_node = goal_node->parent;

    while(current_node != start_node){
        path.push_back(current_node);
        current_node = current_node->parent;
    }

    // Reverse path
    std::reverse(path.begin(), path.end());
    std::cout << "[A*] Path length: " << path.size() << std::endl;
    return path;
}

// To prune the path for the waypoint follower
std::vector<mbot_lcm_msgs::pose2D_t> extract_pose_path(std::vector<Node*> nodes, const ObstacleDistanceGrid& distances)
{
    std::vector<mbot_lcm_msgs::pose2D_t> path;
    ////////////////// TODO: Implement your extract_pose_path function //////////////////////////
    // This should turn the node path into a vector of poses (with heading) in the global frame
    // You should prune the path to get a waypoint path suitable for sending to motion controller
    int idx = 0;
    for(auto&& node : nodes){
        mbot_lcm_msgs::pose2D_t pose;

        Point<double> globalPose = grid_position_to_global_position(Point<double>(node->cell), distances);
        pose.x = globalPose.x;
        pose.y = globalPose.y;

        if(idx == 0){
            pose.theta = 0.0;
        }
        else{
            double dx = pose.x - path[idx-1].x;
            double dy = pose.y - path[idx-1].y;
            pose.theta = wrap_to_pi(atan2(dy, dx));
        }

        path.push_back(pose);
        idx++;
    }
    
    return path;
}

bool is_in_list(Node* node, std::vector<Node*> list)
{
    for (auto &&item : list)
    {
        if (*node == *item) return true;
    }
    return false;
}

Node* get_from_list(Node* node, std::vector<Node*> list)
{
    for (auto &&n : list)
    {
        if (*node == *n) return n;
    }
    return NULL;
}

std::vector<Node*> prune_node_path(std::vector<Node*> nodePath)
{
    std::vector<Node*> new_node_path;
    ////////////////// TODO: Optionally implement a prune_node_path function //////////////////////////
    // This should remove points in the path along the same line
    
    new_node_path.push_back(nodePath[0]);

    for(int i=1; i<nodePath.size()-1; i++){
        int dx1 = nodePath[i]->cell.x - nodePath[i-1]->cell.x;
        int dy1 = nodePath[i]->cell.y - nodePath[i-1]->cell.y;
        int dx2 = nodePath[i+1]->cell.x - nodePath[i]->cell.x;
        int dy2 = nodePath[i+1]->cell.y - nodePath[i]->cell.y;

        if (fabs(dx1*dy2 - dx2*dy1) != 0){        // inner product to check if dx1,dy1 and dx2,dy2 parallel
        // if (fabs(dx1*dy2 - dx2*dy1) >= 0.1){        // inner product to check if dx1,dy1 and dx2,dy2 parallel
            if(abs(nodePath[i]->cell.x - new_node_path.back()->cell.x) > 1 ||
               abs(nodePath[i]->cell.y - new_node_path.back()->cell.y) > 1)
            {
                new_node_path.push_back(nodePath[i]);
            }
        }
    }

    new_node_path.push_back(nodePath.back());

    return new_node_path;
}
