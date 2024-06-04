#include <pluginlib/class_list_macros.h>
#include "RRT_STAR/rrt_global_planner.h"
#include <visualization_msgs/MarkerArray.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)


//Default Constructor
namespace rrt_planner {



double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0), search_radius_(0.0){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");

        nh.param("maxsamples", max_samples_, 3000.0);
        nh.param("search_radius", search_radius_, 1.0);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/3.0);  //or any other distance within local costmap

        nh_global.param("resolution", resolution_, 0.032);

        // std::cout << "Parameters: " << max_samples_ << ", " << dist_th_ << ", " << visualize_markers_ << ", " << max_dist_ << std::endl;
        // std::cout << "Local costmap size: " << width << ", " << height << std::endl;
        // std::cout << "Global costmap resolution: " << resolution_ << std::endl;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;

        points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrt_points", 1, true);
        lines_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrt_lines", 1, true);
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);
    std::cout << "Start: " << start_mx << ", " << start_my << std::endl;

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRTStar;
    std::cout << "Computing RRT*" << std::endl;
    bool computed = computeRRTStar(point_start, point_goal, solRRTStar, tree);
    if (computed){        
        getPlan(solRRTStar, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

bool RRTPlanner::computeRRTStar(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol, std::vector<TreeNode*>& tree){
    // Start the timer
    auto start_time = std::chrono::high_resolution_clock::now();

    bool finished = false;

    //Initialize random number generator
    srand(time(NULL)); // seed the random number generator
        
    // Initialize the tree with the starting point in map coordinates
    TreeNode* start_node = new TreeNode(start, 0.0); // cost set to 0
    TreeNode* goal_node = nullptr;
    std::cout << "Start node: " << start_node->getNode()[0] << ", " << start_node->getNode()[1] << std::endl;

    // std::vector<TreeNode*> tree;
    // tree.push_back(start_node);
    int i = 0;

    while(finished == false && i < max_samples_){
        
  
        std::vector<int> random_point = {rand() % costmap_->getSizeInCellsX(), rand() % costmap_->getSizeInCellsY()};
        TreeNode* random_node = new TreeNode(random_point);
        std::cout << "Random node: " << random_node->getNode()[0] << ", " << random_node->getNode()[1] << std::endl;

        //find the closest node in the tree to the random point using neast
        TreeNode* nearest_node = random_node->neast(start_node);
        std::cout << "Nearest node: " << nearest_node->getNode()[0] << ", " << nearest_node->getNode()[1] << std::endl;
        //check if the path is free
        //get node
        std::vector <int> nearest_point = nearest_node->getNode();
        
        double dist = distance(nearest_point[0], nearest_point[1], random_point[0], random_point[1]) * resolution_;
        std::cout << "Distance from random node to nearest node: " << dist << std::endl;

        if(dist > max_dist_){
            std::cout << "Distance greater than max_dist" << std::endl;
            float angle = atan2(random_point[1]-nearest_point[1], random_point[0]-nearest_point[0]);
            // Same direction at distance max_dist_
            random_point[0]= nearest_point[0] + max_dist_ * cos(angle);
            random_point[1]= nearest_point[1] + max_dist_ * sin(angle);
            std::cout << "New random point: " << random_point[0] << ", " << random_point[1] << std::endl;
            // delete random_node;
            random_node = new TreeNode(random_point);
            // tree.push_back(random_node);   
            std:: cout << "New random node: " << random_node->getNode()[0] << ", " << random_node->getNode()[1] << std::endl;
        }

        bool free = obstacleFree(nearest_point[0], nearest_point[1], random_point[0], random_point[1]);
        std::cout << "Free from random point and nearest point: " << free << std::endl;
        if (free){
            // the input to the near function is the first node and the node within we want to find the neighbors
            // change max dist for testing
            // CAMBIAR NOMBRE A FIND NEIGHBORS PARA NO COPIARNOS
            double actual_min_random_cost = nearest_node->getCost() + distance(nearest_point[0], nearest_point[1], random_point[0], random_point[1]);
            std::cout << "Actual random cost: " << actual_min_random_cost << std::endl;
            random_node->setCost(actual_min_random_cost);

            std::cout << "Finding neighbors with search radius: " << search_radius_ << std::endl;
            std::vector<TreeNode*> neighbors = start_node->near(random_node,start_node,max_dist_);            // compute cost
            std::cout << "Number of neighbors: " << neighbors.size() << std::endl;
            
            TreeNode* min_cost_node = chooseParent(random_node, neighbors);
            // std::cout << "Number of nodes in tree after adding the neighbor node with the lowest cost: " << tree.size() << std::endl;

            if(min_cost_node != nullptr){
                double tentative_cost = min_cost_node->getCost() + distance(min_cost_node->getNode()[0], min_cost_node->getNode()[1], random_node->getNode()[0], random_node->getNode()[1]);
                // std::cout << "Random node cost: " << random_node->getCost() << std::endl;
                // std::cout << "Tentavie cost from neigbor: " << tentative_cost << std::endl;
                if(tentative_cost < actual_min_random_cost){
                    std::cout << "Tentative cost less than the previous cost from random to nearest node" << std::endl;
                    random_node->setCost(tentative_cost);                    
                    actual_min_random_cost = tentative_cost;
                    min_cost_node->appendChild(random_node);
                    std::cout << "New neigbor node is: " << min_cost_node->getNode()[0] << ", " << min_cost_node->getNode()[1] << std::endl;
                    updateCosts(random_node,start_node,max_dist_);
                }
            }

            finished = obstacleFree(min_cost_node->getNode()[0], min_cost_node->getNode()[1], goal[0], goal[1]);
            // hay que añadir si la distancia es menos que el threshold?
            if (finished){
                // double cost_goal_node = random_node->getCost() + distance(min_cost_node->getNode()[0], min_cost_node->getNode()[1], goal[0], goal[1]);
                // goal_node = new TreeNode(goal, cost_goal_node); // add goal node
                // goal_node->setParent(min_cost_node);
                // tree.push_back(goal_node);
                sol = random_node->returnSolution();
            	std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!! Finished !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1" << std::endl;
                // quit the loop
                return true;
                
            }
            visualizeTree(tree);
        }
        i++;
        std:: cout << "Iteration: " << i << std::endl;
    }


    // sol = goal_node->returnSolution();
    std::cout << "Solution size: " << sol.size() << std::endl;
    // return false; // not finished?
    return finished;
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}



void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){
    visualization_msgs::MarkerArray markerArray;
    int id = 0;

    // Previous pose to connect lines
    geometry_msgs::PoseStamped prevPose;

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1;
        plan.push_back(pose);

        // Add a Marker for visualization
        // plotPoint(point, "rrtStar_path_sol", id++, 0.4, 0.0, 1.0, 0.0);

        // Add a Line marker between consecutive points
        if (!prevPose.pose.position.x && !prevPose.pose.position.y) {
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
        lineMarker.scale.x = 0.2;  // Adjust the width of the line
        lineMarker.color.a = 1.0;   // Set alpha (transparency) to 1.0 (fully opaque)
        lineMarker.color.r = 0.0;   // Set color (red)
        lineMarker.color.g = 1.0;   // Set color (green)
        lineMarker.color.b = 0.0;   // Set color (blue)

        markerArray.markers.push_back(lineMarker);

        // Update the previous pose
        prevPose = pose;

    }
}

std::vector<TreeNode*> RRTPlanner::findNeighbors(TreeNode* node, double radius, std::vector<TreeNode*>& tree){
    std::vector<TreeNode*> neighbors;
    std::cout << "  entered finding neighbors function" << std::endl;
    std::cout << "  Number of nodes in tree INSIDE FUNCTION: " << tree.size() << std::endl;
    for (TreeNode* other_node: tree){
        double dist = distance(node->getNode()[0], node->getNode()[1], other_node->getNode()[0], other_node->getNode()[1]);
        if (dist*resolution_ <= radius){
            // std::cout << "  Distance to neigbor node: " << dist*resolution_ << std::endl;
            neighbors.push_back(other_node);
        }
    }
    return neighbors;
}

// Function to find the parent node with the lowest cost to a given node
TreeNode* RRTPlanner::chooseParent(TreeNode* random_node, const std::vector<TreeNode*>& near_nodes) {
    TreeNode* min_cost_parent = nullptr;
    double min_cost = std::numeric_limits<double>::infinity();
    double near_x, near_y, rand_x, rand_y,cost;
    
    for (TreeNode* near_node : near_nodes) {        
        if (obstacleFree(near_node->getNode()[0], near_node->getNode()[1], random_node->getNode()[0], random_node->getNode()[1])) {
            near_x = near_node->getNode()[0];
            near_y = near_node->getNode()[1];
            rand_x = random_node->getNode()[0];
            rand_y = random_node->getNode()[1];
            
            cost = near_node->getCost() + distance(near_x,near_y, rand_x ,rand_y)*resolution_;            
            // Additional condition for rewiring
            if (cost < min_cost) {
                // Check if rewiring the tree with the new node as a parent leads to a lower cost for its children
                min_cost = cost;
                min_cost_parent = near_node; }
        }
    }
    return min_cost_parent;
}


void RRTPlanner::updateCosts(TreeNode* random_node,TreeNode* start_node , double max_dist) {
    std::vector<TreeNode*> near_nodes = start_node->near(random_node,start_node,max_dist_);
    double node_x, node_y, rand_x, rand_y,new_cost;
    for (TreeNode* node : near_nodes) {        
        node_x = node->getNode()[0];
        node_y = node->getNode()[1];
        rand_x = random_node->getNode()[0];
        rand_y = random_node->getNode()[1];
        if (obstacleFree(node_x,node_y, rand_x ,rand_y)){
            new_cost = random_node->getCost() + (distance(node_x,node_y, rand_x ,rand_y)*resolution_);
            if (new_cost < node->getCost() ) {
                node->getParent()->removeChild(node);
                random_node->appendChild(node);
                node->setCost(new_cost);
                updateCosts(node,start_node,max_dist_);
            }   
        }
    }
}

void RRTPlanner::rewire(TreeNode* node, std::vector<TreeNode*> neighbors){
    for (TreeNode* neighbor: neighbors){
        double new_cost = node->getCost() + distance(node->getNode()[0], node->getNode()[1], neighbor->getNode()[0], neighbor->getNode()[1]);
        if (new_cost < neighbor->getCost()){
            neighbor->setParent(node);
            neighbor->setCost(new_cost);
        }
    }
}


// void RRTPlanner::plotPoint(const std::vector<int>& point, const std::string& ns, const int id,
//                                 const double size, const double color_r, const double color_g, const double color_b) {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = global_frame_id_;
//     marker.header.stamp = ros::Time::now();
//     marker.ns = ns;
//     marker.id = id;
//     marker.type = visualization_msgs::Marker::SPHERE;
//     marker.action = visualization_msgs::Marker::ADD;
//     costmap_->mapToWorld(static_cast<unsigned int>(point[0]), static_cast<unsigned int>(point[1]),
//                          marker.pose.position.x, marker.pose.position.y);
//     marker.pose.position.z = 0.0;
//     marker.pose.orientation.w = 1.0;
//     marker.scale.x = size;
//     marker.scale.y = size;
//     marker.scale.z = size;
//     marker.color.a = 1.0;
//     marker.color.r = color_r;
//     marker.color.g = color_g;
//     marker.color.b = color_b;

//     visualization_msgs::MarkerArray markerArray;
//     markerArray.markers.push_back(marker);
//     points_pub_.publish(markerArray);
// }

void RRTPlanner::visualizeTree(const std::vector<TreeNode*>& tree) {
    visualization_msgs::MarkerArray marker_array;

    // Create points marker
    visualization_msgs::Marker points_marker;
    points_marker.header.frame_id = global_frame_id_;
    points_marker.header.stamp = ros::Time::now();
    points_marker.ns = "rrt_points";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::Marker::POINTS;
    points_marker.action = visualization_msgs::Marker::ADD;
    points_marker.pose.orientation.w = 1.0;
    points_marker.scale.x = 0.1;
    points_marker.scale.y = 0.1;
    points_marker.color.r = 1.0;
    points_marker.color.a = 1.0;

    // Create lines marker
    visualization_msgs::Marker lines_marker;
    lines_marker.header.frame_id = global_frame_id_;
    lines_marker.header.stamp = ros::Time::now();
    lines_marker.ns = "rrt_lines";
    lines_marker.id = 1;
    lines_marker.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker.action = visualization_msgs::Marker::ADD;
    lines_marker.pose.orientation.w = 1.0;
    lines_marker.scale.x = 0.05;
    lines_marker.color.b = 1.0;
    lines_marker.color.a = 1.0;

    for (const auto& node : tree) {
        geometry_msgs::Point p;
        costmap_->mapToWorld(node->getNode()[0], node->getNode()[1], p.x, p.y);
        p.z = 0.0;
        points_marker.points.push_back(p);

        if (node->getParent()) {
            geometry_msgs::Point parent_p;
            costmap_->mapToWorld(node->getParent()->getNode()[0], node->getParent()->getNode()[1], parent_p.x, parent_p.y);
            parent_p.z = 0.0;
            lines_marker.points.push_back(p);
            lines_marker.points.push_back(parent_p);
        }
    }

    marker_array.markers.push_back(points_marker);
    marker_array.markers.push_back(lines_marker);

    points_pub_.publish(marker_array);
    lines_pub_.publish(marker_array);
}
};

