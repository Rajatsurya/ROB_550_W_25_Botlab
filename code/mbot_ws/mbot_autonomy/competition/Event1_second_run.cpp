#include <planning/motion_planner_server.hpp>
#include <planning/motion_planner.hpp>
#include <lcm/lcm-cpp.hpp>
#include <thread>

#include <mbot/mbot_channels.h>
#include <lcm_channels.h>
#include <mbot_lcm_msgs/planner_request_t.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/twist2D_t.hpp>


int main(int argc, char** argv){
    if(argc != 4){
        std::cerr << "Usage: " << argv[0] << " <goal_x> <goal_y> <goal_theta>" << std::endl;
        return 1;
    }
    mbot_lcm_msgs::pose2D_t pose;
    pose.x = atof(argv[1]);
    pose.y = atof(argv[2]);
    pose.theta = atof(argv[3]);

    MotionPlannerParams planner_params = MotionPlannerParams();
    MotionPlanner planner = MotionPlanner(planner_params);
    lcm::LCM lcmConnection("udpm://239.255.76.67:7667?ttl=0");
    MotionPlannerServer server(lcmConnection, planner);

    // Set the maximum velocities
    mbot_lcm_msgs::twist2D_t maxVel;
    maxVel.vx = 0.6;
    maxVel.vy = 0;
    maxVel.wz = M_PI * 2.0 / 3.0;
    lcmConnection.publish(MBOT_MAX_VEL_CHANNEL, &maxVel);

    // Define a request object before publishing
    mbot_lcm_msgs::planner_request_t request;
    request.goal = pose;
    request.require_plan = 1; // Set to 1 to require A* planning, 0 to use the goal as a path
    lcmConnection.publish(PATH_REQUEST_CHANNEL, &request);


    std::thread serverThread([&server]() {
        server.run();
    });
    while(true){
        lcmConnection.handle();
    }
    serverThread.join();
    return 0;
}