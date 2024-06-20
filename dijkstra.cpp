#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <queue>
#include <map>
#include <set>
#include <climits>
#include <algorithm>
#include <limits>
#include <utility>
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
    std::unordered_map<std::string, std::vector<Edge>> adjList;

    void addEdge(const Edge &edge)
    {
        adjList[edge.startStop].push_back(edge);
    }

    std::vector<Edge> dijkstra(const std::string &start, const std::string &end, int startTime)
    {
        std::unordered_map<std::string, int> minTravelTime;
        std::unordered_map<std::string, Edge> prev;
        std::priority_queue<std::pair<int, std::string>, std::vector<std::pair<int, std::string>>, std::greater<>> pq;

        for (const auto &pair : adjList)
        {
            minTravelTime[pair.first] = std::numeric_limits<int>::max();
        }
        minTravelTime[start] = 0; // Start time
        pq.emplace(startTime, start);

        while (!pq.empty())
        {
            auto [currentTime, currentStop] = pq.top();
            pq.pop();

            if (currentStop == end)
                break;

            for (const auto &edge : adjList[currentStop])
            {
                int departureTime = edge.departureTime >= currentTime ? edge.departureTime : currentTime + (edge.departureTime + 1440 - currentTime) % 1440; // Handles next day's bus if needed.
                int arrivalTime = departureTime + (edge.arrivalTime - edge.departureTime);

                if (arrivalTime < minTravelTime[edge.endStop])
                {
                    minTravelTime[edge.endStop] = arrivalTime;
                    prev[edge.endStop] = edge;             
                    pq.emplace(arrivalTime, edge.endStop); 
                }
            }
        }

        std::vector<Edge> path;
        for (std::string at = end; at != start && prev.find(at) != prev.end(); at = prev[at].startStop)
        {
            path.push_back(prev[at]);
        }
        std::reverse(path.begin(), path.end());

        return path;
    }
};

int convertTimeToMinutes(const std::string &time)
{
    int hours = std::stoi(time.substr(0, 2));
    int minutes = std::stoi(time.substr(3, 2));
    return hours * 60 + minutes;
}

std::string minutesToHHMM(int minutes)
{
    int hours = minutes / 60;
    int mins = minutes % 60;
    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%02d:%02d", hours, mins);
    return std::string(buffer);
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
    std::string startTimeStr = "12:00";

    int startTime = convertTimeToMinutes(startTimeStr);

    auto shortestPath = graph.dijkstra(startStop, endStop, startTime);

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
