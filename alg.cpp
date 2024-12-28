#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <set>
#include <algorithm>
#include <chrono>

struct Node {
    double lon, lat;
    std::vector<std::pair<Node*, double>> neighbors;
};

struct Graph {
    std::vector<Node*> nodes;

    Node* find_or_create_node(double lat, double lon, std::unordered_map<std::string, Node*>& node_map) {
        std::string key = std::to_string(lat) + "," + std::to_string(lon);
        if (node_map.find(key) == node_map.end()) {
            Node* new_node = new Node{lon, lat};
            nodes.push_back(new_node);
            node_map[key] = new_node;
        }
        return node_map[key];
    }

    void load_from_file(const std::string& filename) {
        std::ifstream file(filename);
        std::string line;
        std::unordered_map<std::string, Node*> node_map;

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string from_node, to_nodes;

            std::getline(ss, from_node, ':');
            std::getline(ss, to_nodes);

            double from_lon, from_lat;
            sscanf(from_node.c_str(), "%lf,%lf", &from_lon, &from_lat);
            Node* from = find_or_create_node(from_lat, from_lon, node_map);

            std::stringstream neighbors_stream(to_nodes);
            std::string neighbor_info;
            while (std::getline(neighbors_stream, neighbor_info, ';')) {
                double to_lon, to_lat, weight;
                sscanf(neighbor_info.c_str(), "%lf,%lf,%lf", &to_lon, &to_lat, &weight);
                Node* to = find_or_create_node(to_lat, to_lon, node_map);
                from->neighbors.push_back({to, weight});
            }
        }
    }

    Node* find_closest_node(double lat, double lon) {
        double min_distance = std::numeric_limits<double>::max();
        Node* closest = nullptr;

        for (Node* node : nodes) {
            double distance = std::sqrt(std::pow(node->lat - lat, 2) + std::pow(node->lon - lon, 2));
            if (distance < min_distance) {
                min_distance = distance;
                closest = node;
            }
        }
        return closest;
    }
};

void bfs(Node* start, Node* goal) {
    std::queue<Node*> q;
    std::unordered_map<Node*, Node*> came_from;

    q.push(start);
    came_from[start] = nullptr;

    while (!q.empty()) {
        Node* current = q.front();
        q.pop();

        if (current == goal) break;

        for (auto& neighbor : current->neighbors) {
            Node* next = neighbor.first;
            if (came_from.find(next) == came_from.end()) {
                q.push(next);
                came_from[next] = current;
            }
        }
    }

    if (came_from.find(goal) != came_from.end()) {
        std::vector<Node*> path;
        for (Node* at = goal; at != nullptr; at = came_from[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        for (Node* node : path) {
            std::cout << "(" << node->lat << ", " << node->lon << ") -> ";
        }
        std::cout << "end\n";
    } else {
        std::cout << "Path not found\n";
    }
}

void dfs(Node* start, Node* goal) {
    std::stack<Node*> s;
    std::unordered_map<Node*, Node*> came_from;

    s.push(start);
    came_from[start] = nullptr;

    while (!s.empty()) {
        Node* current = s.top();
        s.pop();

        if (current == goal) break;

        for (auto& neighbor : current->neighbors) {
            Node* next = neighbor.first;
            if (came_from.find(next) == came_from.end()) {
                s.push(next);
                came_from[next] = current;
            }
        }
    }
    

    if (came_from.find(goal) != came_from.end()) {
        std::vector<Node*> path;
        for (Node* at = goal; at != nullptr; at = came_from[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        for (Node* node : path) {
            std::cout << "(" << node->lat << ", " << node->lon << ") -> ";
        }
        std::cout << "end\n";
    } else {
        std::cout << "Path not found\n";
    }
}

void dijkstra(Node* start, Node* goal) {
    std::unordered_map<Node*, double> distances;
    std::unordered_map<Node*, Node*> came_from;
    std::set<std::pair<double, Node*>> priority_queue;

    for (auto& neighbor : start->neighbors) { 
        distances[neighbor.first] = std::numeric_limits<double>::max();
    }
    distances[start] = 0.0;
    priority_queue.insert({0.0, start});

    while (!priority_queue.empty()) {
        Node* current = priority_queue.begin()->second;
        priority_queue.erase(priority_queue.begin());

        if (current == goal) break;

        for (auto& neighbor : current->neighbors) {
            Node* next = neighbor.first;
            double weight = neighbor.second;
            double new_distance = distances[current] + weight; 

            if (new_distance < distances[next]) {
                priority_queue.erase({distances[next], next});
                distances[next] = new_distance;
                came_from[next] = current;
                priority_queue.insert({new_distance, next});
            }
        }
    }

    if (came_from.find(goal) != came_from.end()) {
        std::vector<Node*> path;
        for (Node* at = goal; at != nullptr; at = came_from[at]) {
            path.push_back(at);
        }
        std::reverse(path.begin(), path.end());
        for (Node* node : path) {
            std::cout << "(" << node->lat << ", " << node->lon << ") -> ";
        }
        std::cout << "end\n";
    } else {
        std::cout << "Path not found\n";
    }
}

int main() {
    Graph graph;
    graph.load_from_file("spb_graph.txt");

    double start_lat = 59.920551, start_lon = 30.2985115;
    double goal_lat = 59.9469059, goal_lon = 30.4140936;

    Node* start = graph.find_closest_node(start_lat, start_lon);
    Node* goal = graph.find_closest_node(goal_lat, goal_lon);

    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "BFS Path:\n";
    bfs(start, goal);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    std::cout << "BFS time: " << duration.count() << " seconds\n";

    start_time = std::chrono::high_resolution_clock::now();
    std::cout << "DFS Path:\n";
    dfs(start, goal);
    end_time = std::chrono::high_resolution_clock::now();
    duration = end_time - start_time;
    std::cout << "DFS time: " << duration.count() << " seconds\n";


    start_time = std::chrono::high_resolution_clock::now();
    std::cout << "Dijkstra Path:\n";
    dijkstra(start, goal);
    end_time = std::chrono::high_resolution_clock::now();
    duration = end_time - start_time;
    std::cout << "Dijkstra time: " << duration.count() << " seconds\n";

    return 0;
}
