#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);    
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}


// Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // Find neighbors of the current_node, this populates current_node.neighbors vector. Set visited to true.
  current_node->visited = true;
  current_node->FindNeighbors();
  
  // For each node in the current_node.neighors vector, set the parent, the h_value, and the g_value
  for ( auto node : current_node->neighbors)
  {
	node->parent = current_node;
    node->h_value = CalculateHValue(node);
    node->g_value = node->distance(*current_node) + current_node->g_value;
    // add the neighbor to open_list and set the node's visited attribute to true.
    open_list.emplace_back(node);
    node->visited = true;
  }
}


// Complete the NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the open_list according to the sum of the h and g value
  std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *a, RouteModel::Node *b){return (a->h_value + a->g_value) > (b->h_value + b->g_value); });
  // Create a pointer to the node in the list with the lowest sum.
  RouteModel::Node *lowestSumNode = open_list.back();
  // Remove that node from the open_list and return the pointer
  open_list.pop_back();
  return lowestSumNode;
}


// Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

  // This method should iteratively follow the chain of parents of nodes until the starting node is found.
  while ( current_node != start_node )
  {
    // For each node in the chain, add the distance from the node to its parent to the distance variable.
	distance += current_node->distance(*current_node->parent);
    path_found.push_back(*current_node);
    current_node = current_node->parent;
  }
  	// Add final (start) node and then reverse order
 	path_found.push_back(*current_node);
  	std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// Write the A* Search algorithm here.
void RoutePlanner::AStarSearch() {
  	RouteModel::Node *current_node = nullptr;
  	current_node = start_node;
  	while( current_node != end_node)
    {
      	// Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
    	AddNeighbors(current_node);
    	current_node = NextNode();
  	}
  	// When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
  	std::vector<RouteModel::Node> path_found = ConstructFinalPath(current_node);
  	// Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
  	m_Model.path = path_found;
}