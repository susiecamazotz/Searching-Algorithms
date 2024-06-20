#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <map>
#include <climits>
#include <algorithm>
#include <limits>
#include <utility>
#include <cmath>

struct Edge
{
    int id;
    std::string company;
    std::string line;
    int departureTime;
    int arrivalTime;
    std::string startStop;
    std::string endStop;
    float startStopLat;
    float startStopLon;
    float endStopLat;
    float endStopLon;
};

class Graph
{
public:
    // Manhattan distance heuristic function
    double heuristic(const std::string &current, const std::string &goal, const Graph &graph)
    {
        double currentX = graph.adjList.at(current).front().startStopLon;
        double currentY = graph.adjList.at(current).front().startStopLat;
        double goalX = graph.adjList.at(goal).front().startStopLon;
        double goalY = graph.adjList.at(goal).front().startStopLat;

        return std::abs(currentX - goalX) + std::abs(currentY - goalY); // Manhattan distance
    }

    std::unordered_map<std::string, std::vector<Edge>> adjList;

    void addEdge(const Edge &edge)
    {
        adjList[edge.startStop].push_back(edge);
    }

std::vector<Edge> a_star(const std::string& start, const std::string& end, int startTime) {
    std::unordered_map<std::string, int> transferCount; 
    std::unordered_map<std::string, Edge> cameFrom; 
    std::priority_queue<std::pair<int, std::string>, std::vector<std::pair<int, std::string>>, std::greater<>> openSet;

    for (const auto& pair : this->adjList) {
        transferCount[pair.first] = std::numeric_limits<int>::max();
    }

    transferCount[start] = 0; // No transfers to start
    openSet.emplace(0, start);

    while (!openSet.empty()) {
        std::string current = openSet.top().second;
        openSet.pop();

        if (current == end) {
            break;
        }

        // Current line taken to reach the 'current' stop
        std::string currentLine = (cameFrom.count(current) > 0) ? cameFrom[current].line : "";

        for (const auto& edge : this->adjList.at(current)) {
            if (edge.departureTime < startTime) continue;

            int newTransferCount = transferCount[current] + (edge.line != currentLine ? 1 : 0); // Increase if line changes
            if (newTransferCount < transferCount[edge.endStop]) {
                transferCount[edge.endStop] = newTransferCount;
                cameFrom[edge.endStop] = edge;
                openSet.emplace(newTransferCount, edge.endStop);
            }
        }
    }

    std::vector<Edge> path;
    for (std::string at = end; at != start && cameFrom.find(at) != cameFrom.end(); at = cameFrom[at].startStop) {
        path.push_back(cameFrom[at]);
    }
    std::reverse(path.begin(), path.end());

    return path;
}

};

int convertTimeToMinutes(const std::string &time)
{
    int hours, minutes, seconds;
    char colon;
    std::istringstream ss(time);
    ss >> hours >> colon >> minutes >> colon >> seconds;
    return hours * 60 + minutes;
}

std::string minutesToHHMM(int minutes)
{
    int hours = minutes / 60;
    int mins = minutes % 60;
    std::ostringstream oss;
    oss << (hours < 10 ? "0" : "") << hours << ':' << (mins < 10 ? "0" : "") << mins;
    return oss.str();
}

void loadEdgesFromCSV(Graph &graph, const std::string &filename)
{
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header

    while (std::getline(file, line))
    {
        std::istringstream s(line);
        std::vector<std::string> fields;
        std::string field;
        while (getline(s, field, ','))
        {
            fields.push_back(field);
        }

        Edge edge;
        edge.id = std::stoi(fields[0]);
        edge.company = fields[1];
        edge.line = fields[2];
        edge.departureTime = convertTimeToMinutes(fields[3]);
        edge.arrivalTime = convertTimeToMinutes(fields[4]);
        edge.startStop = fields[5];
        edge.endStop = fields[6];
        edge.startStopLat = std::stof(fields[7]);
        edge.startStopLon = std::stof(fields[8]);
        edge.endStopLat = std::stof(fields[9]);
        edge.endStopLon = std::stof(fields[10]);

        graph.addEdge(edge);
    }
}

int main()
{
    Graph graph;
    const std::string filename = "connection_graph.csv";

    loadEdgesFromCSV(graph, filename);

    // Example run
    std::string startStop = "Borowska (Szpital)";
    std::string endStop = "PL. GRUNWALDZKI";
    std::string startTimeStr = "12:00";

    int startTime = convertTimeToMinutes(startTimeStr);

    auto shortestPath = graph.a_star(startStop, endStop, startTime);

    // Output
    std::cout << "Shortest path from \"" << startStop << "\" to \"" << endStop
              << "\" starting at " << startTimeStr << ":\n";
    for (const auto &edge : shortestPath)
    {
        std::cout << "Start: " << edge.startStop
                  << ", End: " << edge.endStop
                  << ", Line: " << edge.line
                  << ", Departure: " << minutesToHHMM(edge.departureTime)
                  << ", Arrival: " << minutesToHHMM(edge.arrivalTime) << std::endl;
    }

    return 0;
}
