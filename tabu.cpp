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
#include <list>

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

class Graph {
public:
    std::unordered_map<std::string, std::vector<Edge>> adjList;

    void addEdge(const Edge& edge) {
        adjList[edge.startStop].push_back(edge);
    }

    std::vector<Edge> tabu_search(const std::string& start, const std::string& end, int startTime, int tabuTenure, int maxIterations) {
        std::vector<Edge> bestSolution;
        double bestCost = std::numeric_limits<double>::max();
        std::list<std::vector<Edge>> tabuList;

        std::vector<Edge> currentSolution = generate_initial_solution(start, end, startTime);
        double currentCost = evaluate_solution(currentSolution);

        for (int i = 0; i < maxIterations; ++i) {
            std::vector<Edge> candidateSolution = generate_neighbor(currentSolution);
            double candidateCost = evaluate_solution(candidateSolution);

            if (!is_in_tabu_list(candidateSolution, tabuList) || candidateCost < bestCost) {
                currentSolution = candidateSolution;
                currentCost = candidateCost;

                if (currentCost < bestCost) {
                    bestSolution = currentSolution;
                    bestCost = currentCost;
                }
            }

            update_tabu_list(tabuList, candidateSolution, tabuTenure);
        }

        return bestSolution;
    }

private:
    std::vector<Edge> generate_initial_solution(const std::string& start, const std::string& end, int startTime) {
        // Placeholder implementation
        std::vector<Edge> solution;
        // Generate initial solution logic here
        return solution;
    }

    double evaluate_solution(const std::vector<Edge>& solution) {
        // Placeholder implementation
        if (solution.empty()) return std::numeric_limits<double>::max();
        double totalCost = 0;
        // Evaluate solution logic here
        return totalCost;
    }

    std::vector<Edge> generate_neighbor(const std::vector<Edge>& solution) {
        // Placeholder implementation
        std::vector<Edge> neighbor = solution;
        // Generate neighbor solution logic here
        return neighbor;
    }

bool Graph::is_in_tabu_list(const std::vector<Edge>& solution, const std::list<std::vector<Edge>>& tabuList) {
    // A simple check could be to compare each solution in the tabu list with the current solution
    for (const auto& tabuSolution : tabuList) {
        if (solution == tabuSolution) { // This requires operator== to be implemented for vector<Edge>
            return true;
        }
    }
    return false;
}


void Graph::update_tabu_list(std::list<std::vector<Edge>>& tabuList, const std::vector<Edge>& solution, int tabuTenure) {
    tabuList.push_back(solution);
    while (tabuList.size() > static_cast<size_t>(tabuTenure)) {
        tabuList.pop_front(); // Remove the oldest solution
    }
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

    // Improve I/O performance
    std::ios_base::sync_with_stdio(false);
    file.tie(nullptr);

    std::getline(file, line); // Skip header

    while (std::getline(file, line))
    {
        std::istringstream s(line);
        Edge edge;
        char delim;

        s >> edge.id >> delim;
        std::getline(s, edge.company, ',');
        std::getline(s, edge.line, ',');

        std::string time;
        std::getline(s, time, ',');
        edge.departureTime = convertTimeToMinutes(time);

        std::getline(s, time, ',');
        edge.arrivalTime = convertTimeToMinutes(time);

        std::getline(s, edge.startStop, ',');
        std::getline(s, edge.endStop, ',');

        std::string lat, lon;
        std::getline(s, lat, ',');
        edge.startStopLat = std::stof(lat);

        std::getline(s, lon, ',');
        edge.startStopLon = std::stof(lon);

        std::getline(s, lat, ',');
        edge.endStopLat = std::stof(lat);

        std::getline(s, lon);
        edge.endStopLon = std::stof(lon);

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
