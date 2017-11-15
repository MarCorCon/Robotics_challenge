#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <unordered_map>
#include <unordered_set>

struct AStarPoint
{
	double x;
	double y;
};

class AStar
{
public:
	AStar()   {}
	void addNode(int id, double x, double y);
	void addEdge(int id0, int id1);
	void clear();
	std::list<AStarPoint>& getPath(int start_id, int goal_id, std::list<AStarPoint>& path);
private:
	struct Node
	{
		int id;
		double x;
		double y;
		std::unordered_map<int,double> neighbors;
		double gScore;
		double fScore;
		Node *cameFrom;
	};

	void addEdge(int id0, int id1, double cost);

	double heuristic_cost_estimate(const Node* n0, const Node* n1) const
	{
		return std::sqrt((n0->x - n1->x) * (n0->x - n1->x) + (n0->y - n1->y)* (n0->y - n1->y));
	}
	std::unordered_map<int, Node> nodes;
};


inline
void AStar::clear()
{
	nodes.clear();
}

inline
void AStar::addNode(int id, double x, double y)
{
	Node &n = nodes[id];
	n.id = id;	
	n.x = x;
	n.y = y;
}


inline
void AStar::addEdge(int id0, int id1)
{
	Node& src = nodes.at(id0);
	Node& dst = nodes.at(id1);	
	double cost = heuristic_cost_estimate(&src, &dst);
	addEdge(id0,id1,cost);
	addEdge(id1,id0,cost);
}

inline
void AStar::addEdge(int id0, int id1, double cost)
{
	nodes[id0].neighbors[id1]=cost;	
}


inline
std::list<AStarPoint>& AStar::getPath(int start_id, int goal_id, std::list<AStarPoint>& path)
{
	Node* start_node = &nodes.at(start_id);
	Node* goal_node = &nodes.at(goal_id);
	path.clear();
	std::unordered_set<int> closedSet;
	std::unordered_map<int,Node*> openSet;
	for (auto it = nodes.begin(); it != nodes.end(); ++it) {
		it->second.gScore = std::numeric_limits<double>::infinity();
		it->second.fScore = std::numeric_limits<double>::infinity();
		it->second.cameFrom = NULL;
	}
	
	start_node->gScore = 0;
	start_node->fScore = heuristic_cost_estimate(start_node,goal_node);	
	openSet[start_id] = start_node;	
	while (!openSet.empty()) {
		double min = std::numeric_limits<double>::infinity();
		auto current_it = openSet.begin();
		for (auto it = openSet.begin(); it != openSet.end(); ++it) {
			if (it->second->fScore < min) {
				current_it = it;
				min = it->second->fScore;
			}
		}
		int current_id = current_it->first;
		
		Node* current_node = current_it->second;		
		if (current_id == goal_id) {
			AStarPoint current_point;
			current_point.x = current_node->x;
			current_point.y = current_node->y;
			path.push_front(current_point);
			while (current_node->cameFrom != NULL) {
				current_node = current_node->cameFrom;
				current_point.x = current_node->x;
				current_point.y = current_node->y;
				path.push_front(current_point);
			}
			return path;
		}
		openSet.erase(current_it);		
		closedSet.insert(current_id);
		for (auto it = current_node->neighbors.begin(); it != current_node->neighbors.end(); ++it) {
			int neighbor_id = it->first;
			if (closedSet.count(neighbor_id)>0) {
				continue;
			}
			double neighbor_cost = it->second;
			double tentative_gScore = current_node->gScore + neighbor_cost;
			Node *neighbor_node = &nodes.at(neighbor_id);
			if (openSet.count(neighbor_id)==0) {
				openSet[neighbor_id] = neighbor_node;
			} else if (tentative_gScore >= neighbor_node->gScore) {
				continue;
			}
			neighbor_node->cameFrom = current_node;
			neighbor_node->gScore = tentative_gScore;
			neighbor_node->fScore = tentative_gScore + heuristic_cost_estimate(neighbor_node,goal_node);
		}
	}	
	return path;
}





#endif
