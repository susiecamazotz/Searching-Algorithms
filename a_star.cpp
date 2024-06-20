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
#include <chrono>
#include <chrono>

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

    std::vector<Edge> a_star(const std::string &start, const std::string &end, int startTime)
    {
        std::unordered_map<std::string, double> gScore;
        std::unordered_map<std::string, double> fScore;
        std::unordered_map<std::string, Edge> cameFrom;
        std::priority_queue<std::pair<double, std::string>, std::vector<std::pair<double, std::string>>, std::greater<>> openSet;

        // Initialize scores with infinity
        for (const auto &pair : this->adjList)
        {
            gScore[pair.first] = std::numeric_limits<double>::max();
            fScore[pair.first] = std::numeric_limits<double>::max();
        }

        gScore[start] = 0;
        fScore[start] = heuristic(start, end, *this);
        openSet.emplace(fScore[start], start);

        double currentTime = static_cast<double>(startTime);

        while (!openSet.empty())
        {
            std::string current = openSet.top().second;
            currentTime = std::max(currentTime, openSet.top().first); // Update currentTime to be at least the startTime
            openSet.pop();

            if (current == end)
            {
                break;
            }

            for (const auto &edge : this->adjList.at(current))
            {
                if (edge.departureTime >= currentTime)
                {
                    double travelTime = edge.arrivalTime - edge.departureTime;
                    double waitTime = edge.departureTime - currentTime; // Calculate the waiting time if any
                    double tentative_gScore = currentTime + waitTime + travelTime;

                    if (tentative_gScore < gScore[edge.endStop])
                    {
                        cameFrom[edge.endStop] = edge;
                        gScore[edge.endStop] = tentative_gScore;
                        fScore[edge.endStop] = tentative_gScore + heuristic(edge.endStop, end, *this);
                        openSet.emplace(fScore[edge.endStop], edge.endStop);
                    }
                }
            }
        }

        // Reconstruct the path from end to start
        std::vector<Edge> total_path;
        std::string current = end;
        while (current != start && cameFrom.find(current) != cameFrom.end())
        {
            Edge edge = cameFrom[current];
            total_path.push_back(edge);
            current = edge.startStop;
        }
        std::reverse(total_path.begin(), total_path.end());

        return total_path;
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
    auto start = std::chrono::high_resolution_clock::now();

    Graph graph;
    const std::string filename = "connection_graph.csv";

    loadEdgesFromCSV(graph, filename);

    // Example run
    std::string startStop = "Borowska (Szpital)";
    std::string endStop = "PL. GRUNWALDZKI";
    std::string startTimeStr = "00:10";

    int startTime = convertTimeToMinutes(startTimeStr);

    auto shortestPath = graph.a_star(startStop, endStop, startTime);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

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

    std::cout << "Execution time: " << duration.count() << " milliseconds\n";

    return 0;
}
