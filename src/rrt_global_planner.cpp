#include <pluginlib/class_list_macros.h>
#include "RRT_STAR/rrt_global_planner.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

// Default Constructor
namespace rrt_planner
{

    double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1)
    {
        return std::sqrt((int)(x1 - x0) * (int)(x1 - x0) + (int)(y1 - y0) * (int)(y1 - y0));
    }

    RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                               max_samples_(0.0) {}

    RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {

        if (!initialized_)
        {
            ros::NodeHandle nh("~/" + name);
            ros::NodeHandle nh_local("~/local_costmap/");
            ros::NodeHandle nh_global("~/global_costmap/");

            nh.param("maxsamples", max_samples_, 0.0);
            // nh.param("search_radius", search_radius_, 1.0);

            // to make sure one of the nodes in the plan lies in the local costmap
            double width, height;
            nh_local.param("width", width, 3.0);
            nh_local.param("height", height, 3.0);
            max_dist_ = (std::min(width, height) / 3.0); // change this values to test the algorithm
            search_radius_ = (std::min(width, height) / 3.0); // change this values to test the algorithm

            nh_global.param("resolution", resolution_, 0.032);

            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros->getCostmap();
            global_frame_id_ = costmap_ros_->getGlobalFrameID();

            initialized_ = true;

            lines_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrt_lines", 1, true);
        }
        else
        {
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal,
                              std::vector<geometry_msgs::PoseStamped> &plan)
    {

        std::cout << "RRTPlanner::makePlan" << std::endl;

        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized.");
            return false;
        }

        if (start.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
                      global_frame_id_.c_str(), start.header.frame_id.c_str());
            return false;
        }

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
                      global_frame_id_.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        plan.clear();
        costmap_ = costmap_ros_->getCostmap(); // Update information from costmap

        // Get start and goal poses in map coordinates
        unsigned int goal_mx, goal_my, start_mx, start_my;
        if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my))
        {
            ROS_WARN("Goal position is out of map bounds.");
            return false;
        }
        costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
        std::cout << "Start: " << start_mx << ", " << start_my << std::endl;

        std::vector<int> point_start{(int)start_mx, (int)start_my};
        std::vector<int> point_goal{(int)goal_mx, (int)goal_my};
        std::vector<std::vector<int>> solRRTStar;
        std::cout << "Computing RRT*" << std::endl;
        bool computed = computeRRTStar(point_start, point_goal, solRRTStar);
        if (computed)
        {
            getPlan(solRRTStar, plan);
            // add goal
            plan.push_back(goal);
        }
        else
        {
            ROS_WARN("No plan computed");
        }

        return computed;
    }

    bool RRTPlanner::computeRRTStar(const std::vector<int> start, const std::vector<int> goal,
                                    std::vector<std::vector<int>> &sol)
    {
        // Start the timer
        auto start_time = std::chrono::high_resolution_clock::now();
        std:: cout << "max_dist: " << max_dist_ << std::endl;

        bool finished = false;

        // Initialize random number generator
        srand(time(NULL)); // seed the random number generator

        // Initialize the tree with the starting point in map coordinates
        TreeNode *start_node = new TreeNode(start); // cost set to 0
        TreeNode *goal_node = nullptr;
        TreeNode *random_node;
        std::cout << "Start node: " << start_node->getNode()[0] << ", " << start_node->getNode()[1] << std::endl;
        std::cout << "Goal node: " << goal[0] << ", " << goal[1] << std::endl;

        int i = 0;

        while (finished == false && i < max_samples_)
        {

            std::vector<int> random_point = {rand() % costmap_->getSizeInCellsX(), rand() % costmap_->getSizeInCellsY()};
            random_node = new TreeNode(random_point);
            // std::cout << "Random node: " << random_node->getNode()[0] << ", " << random_node->getNode()[1] << std::endl;

            // find the closest node in the tree to the random point using neast
            TreeNode *nearest_node = random_node->neast(start_node);
            // std::cout << "Nearest node: " << nearest_node->getNode()[0] << ", " << nearest_node->getNode()[1] << std::endl;
            // get node
            std::vector<int> nearest_point = nearest_node->getNode();

            double dist = distance(nearest_point[0], nearest_point[1], random_point[0], random_point[1]) * resolution_;

            if (dist > max_dist_)
            {
                // std::cout << "Distance greater than max_dist" << std::endl;
                float angle = atan2((random_point[1] - nearest_point[1]), (random_point[0] - nearest_point[0]));
                // Same direction at distance max_dist_
                random_point[0] = std::cos(angle) * max_dist_;
                random_point[1] = std::sin(angle) * max_dist_;
                // std::cout << "New random point: " << random_point[0] << ", " << random_point[1] << std::endl;
                random_node = new TreeNode(random_point);
                // std:: cout << "New random node: " << random_node->getNode()[0] << ", " << random_node->getNode()[1] << std::endl;
            }

            bool free = obstacleFree(nearest_point[0], nearest_point[1], random_point[0], random_point[1]);
            // std::cout << "Free from random point and nearest point: " << free << std::endl;
            if (free)
            {
                // the input to the near function is the first node and the node within we want to find the neighbors
                // change radius for testings

                // std::cout << "Finding neighbors with search radius: " << max_dist_ << std::endl;
                std::vector<TreeNode *> neighbors = start_node->findNeighbors(random_node, start_node, search_radius_);
                // std::cout << "Number of neighbors: " << neighbors.size() << std::endl;

                // find the parent with the lowest cost
                TreeNode *best_parent = nullptr;
                double min_tentative_cost = std::numeric_limits<double>::infinity();
                double cost;

                for (TreeNode *near_node : neighbors)
                {
                    if (obstacleFree(near_node->getNode()[0], near_node->getNode()[1], random_node->getNode()[0], random_node->getNode()[1]))
                    {

                        cost = near_node->getCost() + distance(near_node->getNode()[0], near_node->getNode()[1], random_node->getNode()[0], random_node->getNode()[1]) * resolution_;
                        if (cost < min_tentative_cost)
                        {
                            min_tentative_cost = cost;
                            best_parent = near_node;
                        }
                    }
                }

                if (best_parent)
                {
                    double new_cost = best_parent->getCost() + distance(best_parent->getNode()[0], best_parent->getNode()[1], random_node->getNode()[0], random_node->getNode()[1]) * resolution_;
                    // std::cout << "Random node cost: " << random_node->getCost() << std::endl;
                    // std::cout << "Tentative cost from neighbor: " << tentative_cost << std::endl;
                    random_node->setCost(new_cost);
                    best_parent->appendChild(random_node);
                    // std::cout << "New neigbor node is: " << min_cost_node->getNode()[0] << ", " << min_cost_node->getNode()[1] << std::endl;
                    rewire(random_node, start_node, search_radius_);
                }

                double goal_distance = distance(best_parent->getNode()[0], best_parent->getNode()[1], goal[0], goal[1]) * resolution_;
                bool goal_free = obstacleFree(best_parent->getNode()[0], best_parent->getNode()[1], goal[0], goal[1]);
                if (goal_distance <= 1 && goal_free)
                {
                    sol = random_node->returnSolution();
                    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!! Finished !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << std::endl;
                    // quit the loop

                    finished = true;
                }
            }
            i++;
            // std:: cout << "Iteration: " << i << std::endl;
        }

        // Stop the timer
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate elapsed time in milliseconds
        auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double elapsed_time_ms = static_cast<double>(elapsed_time.count()) / 1000.0;
        std::cout << "Time (ms): " << elapsed_time_ms << std::endl;

        // Print the elapsed time in milliseconds
        ROS_INFO("RRTStar computation time: %.4f milliseconds", elapsed_time_ms);

        auto cost = random_node->getCost() + (distance(random_node->getNode()[0], random_node->getNode()[1], goal[0], goal[1]) * resolution_);
        std::cout << "Cost of the path: " << cost << std::endl;

        // print the distance from the start to goal
        std::cout << "Number of nodes in path" << sol.size() << std::endl;
        std::cout << "Density of the tree: " << i << std::endl;

        // return false; // not finished?
        return finished;
    }

    bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0,
                                  const unsigned int x1, const unsigned int y1)
    {
        // Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

        int dx = x1 - x0;
        int dy = y1 - y0;

        int incr_x = (dx > 0) ? 1.0 : -1.0;
        int incr_y = (dy > 0) ? 1.0 : -1.0;

        unsigned int da, db, incr_x_2, incr_y_2;
        if (abs(dx) >= abs(dy))
        {
            da = abs(dx);
            db = abs(dy);
            incr_x_2 = incr_x;
            incr_y_2 = 0;
        }
        else
        {
            da = abs(dy);
            db = abs(dx);
            incr_x_2 = 0;
            incr_y_2 = incr_y;
        }

        int p = 2 * db - da;
        unsigned int a = x0;
        unsigned int b = y0;
        unsigned int end = da;
        for (unsigned int i = 0; i < end; i++)
        {
            if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE)
            { // to include cells with inflated cost
                return false;
            }
            else
            {
                if (p >= 0)
                {
                    a += incr_x;
                    b += incr_y;
                    p -= 2 * da;
                }
                else
                {
                    a += incr_x_2;
                    b += incr_y_2;
                }
                p += 2 * db;
            }
        }

        return true;
    }

    void RRTPlanner::getPlan(const std::vector<std::vector<int>> &sol, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        visualization_msgs::MarkerArray markerArray;
        int id = 0;

        // Previous pose to connect lines
        geometry_msgs::PoseStamped prevPose;

        for (auto it = sol.rbegin(); it != sol.rend(); ++it)
        {
            std::vector<int> point = *it;
            geometry_msgs::PoseStamped pose;

            costmap_->mapToWorld(static_cast<unsigned int>(point[0]), static_cast<unsigned int>(point[1]),
                                 pose.pose.position.x, pose.pose.position.y);
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = global_frame_id_;
            pose.pose.orientation.w = 1;
            plan.push_back(pose);

            std::cout << "Point outside: " << point[0] << ", " << point[1] << std::endl;
            
            // Add a Line marker between consecutive points
            if (!prevPose.pose.position.x && !prevPose.pose.position.y)
            {
                // Skip connecting the first point (no previous point)
                prevPose = pose;
                continue;
            }
            visualization_msgs::Marker lineMarker;
            lineMarker.header.frame_id = global_frame_id_;
            lineMarker.header.stamp = ros::Time::now();
            lineMarker.ns = "rrtStar_path_lines_sol";
            lineMarker.id = id++;
            lineMarker.type = visualization_msgs::Marker::LINE_STRIP;
            lineMarker.action = visualization_msgs::Marker::ADD;
            lineMarker.points.push_back(prevPose.pose.position);
            lineMarker.points.push_back(pose.pose.position);
            lineMarker.scale.x = 0.2; 
            lineMarker.color.a = 1.0; 
            lineMarker.color.r = 0.0; 
            lineMarker.color.g = 1.0; 
            lineMarker.color.b = 0.0; 

            markerArray.markers.push_back(lineMarker);

            // Update the previous pose
            prevPose = pose;

        }

        // Publish the MarkerArray outside the loop
        lines_pub_.publish(markerArray);
    }

    void RRTPlanner::rewire(TreeNode *random_node, TreeNode *start_node, double search_radius_)
    {
        std::vector<TreeNode *> neighbors = start_node->findNeighbors(random_node, start_node, search_radius_);
        double new_cost;
        for (TreeNode *neighbor : neighbors)
        {
            if (obstacleFree(neighbor->getNode()[0], neighbor->getNode()[1], random_node->getNode()[0], random_node->getNode()[1]))
            {
                new_cost = random_node->getCost() + (distance(neighbor->getNode()[0], neighbor->getNode()[1], random_node->getNode()[0], random_node->getNode()[1]) * resolution_);
                if (new_cost < neighbor->getCost())
                {
                    neighbor->getParent()->removeChild(neighbor);
                    random_node->appendChild(neighbor);
                    neighbor->setCost(new_cost);
                    rewire(neighbor, start_node, max_dist_);
                }
            }
        }
    }

};
