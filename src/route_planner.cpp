#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Init RoutePlanner's start_node and end_node attributes (pointer) by using the m_Model.FindClosestNode method 
    // to find the closest nodes to the given starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


/**
 * @brief Calculate the heuristic value for a given node.
 * 
 * This function calculates the heuristic value (H-value) for the provided node
 * by computing the distance from the node to the end node. The heuristic value
 * is used in pathfinding algorithms to estimate the cost to reach the goal.
 * 
 * @param *node: A pointer to the given node.
 * @return float: The heuristic value, representing the distance from the given node to the end node.
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


/**
 * @brief Adds all neighbors of the current node to the open list.
 *
 * This function finds all the neighbors of the given current node, sets their parent to the current node,
 * calculates their h_value using the CalculateHValue function, updates their g_value based on the distance
 * from the current node, and adds them to the open list. It also marks each neighbor as visited.
 *
 * @param RouteModel::Node*: current_node A pointer to the current node whose neighbors are to be added.
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto * neighbor: current_node->neighbors) {
        // check if the neighbor node is visited to prevent re-adding
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            open_list.push_back(neighbor);
            neighbor->visited = true;
        }
    }
}


/**
 * @brief Compares the f-values of two nodes.
 *
 * The f-value is the sum of the h-value (heuristic value) and the g-value (cost from the start node).
 * This function returns true if the f-value of the first node is less than the f-value of the second node.
 *
 * @param RouteModel::Node*: node_1 Pointer to the first node.
 * @param RouteModel::Node*: node_2 Pointer to the second node.
 * @return bool: true if the f-value of node_1 is less than the f-value of node_2, false otherwise.
 */
bool CompareFValue(const RouteModel::Node *node_1, const RouteModel::Node *node_2) {
    return (node_1->h_value + node_1->g_value) < (node_2->h_value + node_2->g_value);
}

/**
 * @brief Selects the next node to process from the open list.
 *
 * This function sorts the open list of nodes based on their f-value using the CompareFValue function.
 * It then selects the node with the lowest f-value, removes it from the open list, and returns it.
 *
 * @return RouteModel::Node* lowest_sum_node A Pointer to the node with the lowest f-value in the open list.
 */
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), CompareFValue);
    RouteModel::Node *lowest_sum_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_sum_node;
}



/**
 * @brief Constructs the final path found by A* search.
 *
 * This method iteratively follows the chain of parent nodes starting from the end node
 * (current node) until the start node is reached. It constructs the path by 
 * pushing each node onto a vector and calculating the total distance. The 
 * resulting path is then reversed to ensure the start node is the first element 
 * and the end node is the last element.
 *
 * @param RouteModel::Node*: current_node A pointer to the current node from which to start constructing the path.
 * @return std::vector<RouteModel::Node> path_found A vector of RouteModel::Node objects representing the final path from the start node to the end node.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    // iteratively follow the chain of parents of nodes until the starting node is found.
    while(current_node != start_node){
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    path_found.push_back(*start_node);
    // reverse the path_found vector to get correct order:
    // the start node should be the first element of the vector, the end node should be the last element.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


/**
 * @brief Performs the A* Search algorithm to find the shortest path from the start node to the end node.
 * 
 * This function uses the A* Search algorithm to navigate through the nodes of the route model.
 * It continues to explore nodes until there are no new nodes to explore. 
 * 
 * During the search, it selects the next node to explore based on the lowest
 * cost path. If the end node is reached, it constructs the final path from the start node to the
 * end node and assigns it to the model's path. If not, it adds neighbors
 * of the current node to the open list and continues the search.
 */
void RoutePlanner::AStarSearch() {
    // initialize the starting node
    // ensures that the algorithm starts correctly and does not revisit the start_node.
    RouteModel::Node *current_node = start_node;
    current_node->visited = true;
    open_list.push_back(start_node);

    while(!open_list.empty()){    
        current_node = NextNode();

        // Check if end_node was reached
        if (current_node == end_node) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } 

        // If end_node was not reached, expand search to current node's neighbors
        AddNeighbors(current_node);
    }
    // We've run out of new nodes to explore and haven't found a path.
    std::cout << "No path found!" << "\n";

}