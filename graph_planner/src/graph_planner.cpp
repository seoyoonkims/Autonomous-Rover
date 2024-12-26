#include <ros/ros.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <climits>
#include <queue>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <graph_planner/graph_planner.h>

GraphPlanner::GraphPlanner(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.getParam("/graph_planner_node/csv_path", csv_path);
    nh_.getParam("/graph_planner_node/robot_postion_topic", robot_postion_topic);
    nh_.getParam("/graph_planner_node/use_rviz_goal", bUseRvizGoal);

    sub_robot_position = nh_.subscribe(robot_postion_topic, 1, &GraphPlanner::RobotPositionCB, this);
    sub_goal_position = nh_.subscribe("/move_base_simple/goal", 1, &GraphPlanner::GoalPositionCB, this);
    sub_global_plan_trigger = nh_.subscribe("/bool/global_plan/trigger", 1, &GraphPlanner::GlobalPlanTriggerCB, this);

    pub_global_path = nh_.advertise<nav_msgs::Path>("/graph_planner/path/global_path", 1);
    pub_node_points = nh_.advertise<sensor_msgs::PointCloud2>("/graph_planner/points/node_points", 1);

    node_pub = nh_.advertise<visualization_msgs::MarkerArray>("/graph_planner/marker/nodes", 1);
    node_text_pub = nh_.advertise<visualization_msgs::MarkerArray>("/graph_planner/marker/nodes_text", 1);
    edge_pub = nh.advertise<visualization_msgs::Marker>("/graph_planner/marker/edges", 1);

    new_graph_pub = nh.advertise<std_msgs::Bool>("/bool/graph_planner/done", 1);

    LoadCSV(csv_path);
    UpdateAllEdgeWeight();
}
GraphPlanner::~GraphPlanner()
{
}

double GraphPlanner::calculateDistance(double x1, double y1, double x2, double y2)
{
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int GraphPlanner::findClosestNode(const geometry_msgs::PoseStamped &pose, const std::vector<Node> &nodes)
{
    double minDistance = DBL_MAX;
    int closestNodeId = -1;

    double x = pose.pose.position.x;
    double y = pose.pose.position.y;

    for (const auto &node : nodes)
    {
        double distance = calculateDistance(x, y, node.x, node.y);
        if (distance < minDistance)
        {
            minDistance = distance;
            closestNodeId = node.id;
        }
    }

    return closestNodeId;
}

void GraphPlanner::RobotPositionCB(const nav_msgs::Odometry &msg)
{
    m_odom = msg;
    geometry_msgs::Pose tmp_pose = msg.pose.pose;
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.pose = tmp_pose;
    start_node = findClosestNode(robot_pose, nodes);
    bGetRobotPose = true;
}

void GraphPlanner::GoalPositionCB(const geometry_msgs::PoseStamped &msg)
{
    geometry_msgs::PoseStamped goal_pose = msg;
    goal_node = findClosestNode(goal_pose, nodes);

    bGetRvizGoal = true;
}

void GraphPlanner::GlobalPlanTriggerCB(const std_msgs::Bool &msg)
{
    bGlobalPlanTrigger = msg.data;
}

void GraphPlanner::PublishGraph()
{
    visualization_msgs::MarkerArray node_markers;
    visualization_msgs::MarkerArray node_markers_text;
    visualization_msgs::Marker edge_marker;

    edge_marker.header.frame_id = "odom";
    edge_marker.header.stamp = ros::Time::now();
    edge_marker.ns = "edges";
    edge_marker.action = visualization_msgs::Marker::ADD;
    edge_marker.pose.orientation.w = 1.0;
    edge_marker.id = 0;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;
    edge_marker.scale.x = 0.02;
    edge_marker.color.r = 1.0;
    edge_marker.color.g = 1.0; // Add green to make the color yellow
    edge_marker.color.a = 1.0;

    for (const auto &node : nodes)
    {
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "odom";
        node_marker.header.stamp = ros::Time::now();
        node_marker.ns = "nodes";
        node_marker.id = node.id;
        node_marker.type = visualization_msgs::Marker::SPHERE;
        node_marker.action = visualization_msgs::Marker::ADD;
        node_marker.pose.position.x = node.x;
        node_marker.pose.position.y = node.y;
        node_marker.pose.position.z = 0;
        node_marker.pose.orientation.w = 1;
        node_marker.scale.x = 0.2;
        node_marker.scale.y = 0.2;
        node_marker.scale.z = 0.2;
        node_marker.color.g = 1.0;
        node_marker.color.a = 1.0;
        node_markers.markers.push_back(node_marker);

        visualization_msgs::Marker node_marker_text;
        node_marker_text.header.frame_id = "odom";
        node_marker_text.header.stamp = ros::Time::now();
        node_marker_text.text = std::to_string(node.id);
        node_marker_text.id = node.id;
        node_marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        node_marker_text.action = visualization_msgs::Marker::ADD;
        node_marker_text.pose.position.x = node.x;
        node_marker_text.pose.position.y = node.y;
        node_marker_text.pose.position.z = 0;
        node_marker_text.pose.orientation.w = 1.0;
        node_marker_text.pose.orientation.x = 0.0;
        node_marker_text.pose.orientation.y = 0.0;
        node_marker_text.pose.orientation.z = 0.0;
        node_marker_text.scale.x = 0.5;
        node_marker_text.scale.y = 0.5;
        node_marker_text.scale.z = 0.5;
        
        if(node.mine)
        {
            node_marker_text.color.r = 1.0;
            node_marker_text.color.a = 1.0;
        }       
        else
        {
            node_marker_text.color.g = 1.0;
            node_marker_text.color.a = 1.0;
        }
        node_markers_text.markers.push_back(node_marker_text);

        for (const auto &edge : graph[node.id - 1])
        {
            geometry_msgs::Point p1, p2;
            p1.x = node.x;
            p1.y = node.y;
            p1.z = 0;

            p2.x = nodes[edge.first - 1].x;
            p2.y = nodes[edge.first - 1].y;
            p2.z = 0;

            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
        }
    }

    node_pub.publish(node_markers);
    node_text_pub.publish(node_markers_text);
    edge_pub.publish(edge_marker);
}

std::vector<Node> GraphPlanner::dijkstra(int start, int target)
{
    // Initialize the distance vector with maximum values
    std::vector<double> dist(nodes.size(), std::numeric_limits<double>::max());
    dist[start - 1] = 0; // Adjusting to 0-based index

    //! initialize prev_node id
    for(int index = 0; index < nodes.size(); index++)
    {
        nodes[index].prev_id = 0;
    }

    // Min-heap priority queue to process nodes based on the current shortest distance
    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
    pq.push({0, start}); // Push the start node with distance 0

    while (!pq.empty())
    {
        double currentDist = pq.top().weight;
        int currentNode = pq.top().node;
        pq.pop();

        if (currentNode == target)
        {
            break; // Stop when the target node is reached
        }

        if (currentDist > dist[currentNode - 1])
        {
            continue;
        }

        for (const auto &edge : graph[currentNode - 1])
        { // Adjusting to 0-based index
            int nextNode = edge.first;
            double weight = edge.second;
            double newDist = currentDist + weight;

            if (newDist < dist[nextNode - 1])
            {
                dist[nextNode - 1] = newDist;
                nodes[nextNode - 1].prev_id = currentNode; // Store the previous node
                pq.push({newDist, nextNode});
            }
        }
    }

    // Reconstruct the shortest path to the target node
    std::vector<Node> path;
    for (int at = target; at != 0; at = nodes[at - 1].prev_id)
    {
        path.push_back(nodes[at - 1]);
    }
    std::reverse(path.begin(), path.end());

    // Check if the start node is in the path
    if (path.empty())
    {
        std::cout << "no path" << std::endl;
        path.clear(); // No path found
    }

    return path;
}

void GraphPlanner::addNode(int id, double x, double y, bool mine)
{
    nodes.push_back({id, 0, x, y, false, mine});
    graph.resize(nodes.size());
}

void GraphPlanner::addEdge(int from, int to, double weight)
{
    graph[from - 1].emplace_back(to, weight); // Adjusting to 0-based index
}

void GraphPlanner::UpdateNode(int id, int prev_id, double x, double y, bool visited, bool mine)
{
    if (id - 1 < nodes.size())
    {
        nodes[id - 1].visited = prev_id;
        nodes[id - 1].x = x;
        nodes[id - 1].y = y;
        nodes[id - 1].visited = visited;
        nodes[id - 1].mine = mine;
    }
    else
    {
        std::cerr << "Node ID " << id << " is out of bounds." << std::endl;
    }
}

void GraphPlanner::UpdateEdgeWeight(int from, int to, double newWeight)
{
    for (auto &edge : graph[from - 1])
    { // Adjusting to 0-based index
        if (edge.first == to)
        {
            edge.second = newWeight;
            return;
        }
    }
    // If the edge doesn't exist, add it
    addEdge(from, to, newWeight);
}

void GraphPlanner::UpdateAllEdgeWeight()
{
    for (size_t from = 0; from < graph.size(); ++from)
    {
        for (auto &edge : graph[from])
        {
            int to = edge.first;
            double x1 = nodes[from].x;
            double y1 = nodes[from].y;
            double x2 = nodes[to - 1].x; // Adjusting to 0-based index
            double y2 = nodes[to - 1].y; // Adjusting to 0-based index
            double newWeight = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
            edge.second = newWeight;
        }
    }
}

int GraphPlanner::findNearestMineNode(int start)
{
    std::vector<std::pair<int, double>> dist;
    for(auto node : nodes)
    {
        int node_id = node.id; //! starts from 1 ~ 16
        dist.push_back(std::make_pair(node_id, std::numeric_limits<double>::max()));
    }

    dist[start - 1].second = 0; // Adjusting to 0-based index

    std::priority_queue<Edge, std::vector<Edge>, std::greater<Edge>> pq;
    pq.push({0, start}); // Push the start node with distance 0

    int nearestMineNode = -1;
    double nearestMineDistance = std::numeric_limits<double>::max();
    int nearestNonMineNode = -1;
    double nearestNonMineDistance = std::numeric_limits<double>::max();

    while (!pq.empty())
    {
        double currentDist = pq.top().weight;
        int currentNode = pq.top().node;
        pq.pop();

        if (nodes[currentNode - 1].mine)
        {
            if (currentDist < nearestMineDistance)
            {
                nearestMineDistance = currentDist;
                nearestMineNode = currentNode;
            }
        }
        else
        {
            if (currentDist < nearestNonMineDistance)
            {
                nearestNonMineDistance = currentDist;
                nearestNonMineNode = currentNode;
            }
        }

        for (const auto &edge : graph[currentNode - 1])
        { // Adjusting to 0-based index
            int nextNode = edge.first;
            double weight = edge.second;

            double newDist = currentDist + weight;

            if (newDist < dist[nextNode - 1].second)
            {
                dist[nextNode - 1].second = newDist;
                pq.push({newDist, nextNode});
            }
        }
    }

    auto compare_distances = [](const std::tuple<int, double>& a, const std::tuple<int, double>& b) 
    {
    return std::get<1>(a) < std::get<1>(b);
    };

    std::sort(dist.begin(), dist.end(), compare_distances);

    std::vector<std::tuple<int, double>> mine_dist_vector;
    std::vector<std::tuple<int, double>> non_mine_dist_vector;
    for(auto dist_buf : dist)
    {
        int node_id = std::get<0>(dist_buf);
        double distance = std::get<1>(dist_buf);

        if(nodes[node_id - 1].mine)
        {
            if(!nodes[node_id - 1].visited)
            {
                mine_dist_vector.push_back(std::make_tuple(node_id, distance));
            }
        }
        else
        {
            if(!nodes[node_id - 1].visited)
            {           
                non_mine_dist_vector.push_back(std::make_tuple(node_id, distance));
            }
        }
    }
    
    for(auto dist_buf : mine_dist_vector)
    {
        int mine_node_id = std::get<0>(dist_buf);
        double mine_dist = std::get<1>(dist_buf);
    }


    if(mine_dist_vector.empty())
    {
        nearestMineNode = std::get<0>(non_mine_dist_vector[0]);
    }
    else
    {
        nearestMineNode = std::get<0>(mine_dist_vector[0]);

        if(std::get<0>(mine_dist_vector[0]) == start)
        {
            nearestMineNode = std::get<0>(mine_dist_vector[1]);
        }
    }

    return nearestMineNode;
}

void GraphPlanner::Plan()
{
    p_nodes.reset(new pcl::PointCloud<pcl::PointXYZI>);

    if (!bGetMineGoal)
    {
        goal_node = findNearestMineNode(start_node);
        ROS_WARN("[MINE SEARCH] START NODE %d, TARGET NODE %d", start_node, goal_node);
        bGetMineGoal = true;
    }

    if (bGetMineGoal && bMineSearchTrigger)
    {
        ROS_WARN_THROTTLE(0.5, "RUN DIJKSTRA"); //! Search only once. No need to run DIJKSTRA algorithm continuously.
        prev_solution.clear();
        std::vector<Node> shortestPath = dijkstra(start_node, goal_node);
        prev_solution = shortestPath;
        bMineSearchTrigger = false;
        
    }   

    //! Decide Arrival mine node
    double arrival_distance = std::sqrt(std::pow(m_odom.pose.pose.position.x - nodes[goal_node - 1].x, 2) + 
                                std::pow(m_odom.pose.pose.position.y - nodes[goal_node - 1].y, 2));

    if(arrival_distance < this->ARRIVAL_THRES)
    {
        ROS_WARN("ARRIVAL MINE NODE");
        nodes[goal_node - 1].visited = true;
        bGetMineGoal = false;
        bMineSearchTrigger = true;
    }

    //! Publish data through nav_msgs::Path type.
    nav_msgs::Path output;

    for (auto node : prev_solution)
    {
        geometry_msgs::PoseStamped pt;
        pt.pose.position.x = node.x;
        pt.pose.position.y = node.y;
        pt.pose.position.z = 0.0;
        pt.pose.orientation.w = 1.0;
        pt.pose.orientation.x = 0.0;
        pt.pose.orientation.y = 0.0;
        pt.pose.orientation.z = 0.0;

        output.poses.push_back(pt);
    }
    output.header.frame_id = "odom";

    pub_global_path.publish(output);
    
    std_msgs::Bool msg;
    msg.data = bMineSearchTrigger;
    new_graph_pub.publish(msg); // modify

    bGetRvizGoal = false;
}

void GraphPlanner::LoadCSV(const std::string &filename)
{
    std::cout << filename << std::endl;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Check file path." << std::endl;
    }

    std::string line;
    std::string header;

    getline(file, header);
    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string id, x, y, start, mine, edgeStr;

        getline(ss, id, ' ');
        getline(ss, x, ' ');
        getline(ss, y, ' ');
        getline(ss, start, ' ');
        getline(ss, mine, ' ');
        getline(ss, edgeStr, ' ');

        int nodeId = std::stoi(id);
        double nodeX = std::stod(x);
        double nodeY = std::stod(y);
        bool mine_check = false;
        // bool mine
        if (mine == "True")
        {
            mine_check = true;
        }

        addNode(nodeId, nodeX, nodeY, mine_check);

        // Process the edge information
        edgeStr.erase(std::remove(edgeStr.begin(), edgeStr.end(), '['), edgeStr.end());
        edgeStr.erase(std::remove(edgeStr.begin(), edgeStr.end(), ']'), edgeStr.end());

        std::stringstream edgeStream(edgeStr);
        std::vector<int> edges;
        std::string edgeValue;

        while (getline(edgeStream, edgeValue, ','))
        {
            edges.push_back(std::stoi(edgeValue));
        }
        for (int neighborId : edges)
        {
            addEdge(nodeId, neighborId, 0.0);
        }
    }
    file.close();
}

void GraphPlanner::run()
{
    PublishGraph();
    if (bGetRobotPose)
    {
        Plan();
    }
    else
    {
        ROS_WARN_THROTTLE(0.5, "bGetRobotPose %d bGetRvizGoal %d", bGetRobotPose, bGetRvizGoal);
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "GraphPlanner");
    ros::NodeHandle _nh("~");

    printf("Initiate: GraphPlanner\n");

    GraphPlanner node_link_loader(_nh);
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        node_link_loader.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    printf("Terminate: GraphPlanner\n");

    return 0;
}
