#include "Algorithms.hpp"
#include "Graph.hpp"
#include <limits.h>
#include <string>
#include <stack>

namespace ariel
{
    // Structure to store a path node with its vertex index and previous node for shortest path
    struct PathNode
    {
        int vertex;
        int prev;
    };

    // Convert a vector of size_t representing a cycle to a string
    vector<size_t> Algorithms::CycleVectrorToString(size_t startNode, size_t endNode, vector<size_t> &parent)
    {
        vector<size_t> cycle;
        for (size_t i = startNode; i != endNode; i = parent[i])
        {
            cycle.push_back(i);
        }
        cycle.push_back(endNode); // Complete the cycle by adding the start node again
        reverse(cycle.begin(), cycle.end());
        return cycle;
    }

    // Utility function for detecting cycles in a graph using DFS
    bool Algorithms::isCyclicUtil(size_t ver, vector<bool> &visited, vector<bool> &recStack, vector<size_t> &parent, Graph &graph, vector<size_t> &cycle)
    {
        visited[ver] = true;
        recStack[ver] = true;
        bool isDirected = graph.isDirected();
        size_t vertixs = graph.getNumVertices();
        vector<vector<int>> GraphMat = graph.getAdjacencyMatrix(); // Get the adjacency matrix of the graph

        for (size_t i = 0; i < vertixs; i++)
        {
            if (GraphMat[ver][i] != 0)
            {
                if (!visited[i])
                {
                    parent[i] = ver;
                    if (isCyclicUtil(i, visited, recStack, parent, graph, cycle))
                    {
                        return true;
                    }
                }
                else if ((isDirected && recStack[i]) || (!isDirected && recStack[i] && parent[ver] != i))
                {
                    cycle = CycleVectrorToString(ver, i, parent);
                    return true;
                }
            }
        }

        recStack[ver] = false;
        return false;
    }

    // Check if a graph contains a cycle
    string Algorithms::isContainsCycle(Graph &graph)
    {
        if (graph.getNumVertices() < 2 || graph.getNumEdges() < 2)
        {
            return "0"; // Return "0" if the graph is empty or has less than 2 edges/vertices
        }

        size_t vertixs = graph.getNumVertices();
        vector<bool> visited(vertixs, false);
        vector<bool> recStack(vertixs, false);
        vector<size_t> parent(vertixs, SIZE_MAX);
        vector<size_t> cycle;
        string cycleStr;

        for (size_t i = 0; i < vertixs; i++)
        {
            if (!visited[i])
            {
                if (isCyclicUtil(i, visited, recStack, parent, graph, cycle))
                {
                    for (size_t j = 0; j < cycle.size(); ++j)
                    {
                        cycleStr += to_string(cycle[j]);
                        if (j != cycle.size() - 1)
                        {
                            cycleStr += "->";
                        }
                    }
                    cycleStr += "->" + to_string(cycle[0]); // Complete the cycle by adding the start node again
                    if (cycleStr.size() > 1)
                    {
                        return cycleStr;
                    }
                }
            }
        }

        cycleStr = "0";
        return cycleStr; // Return "0" if no cycle is found
    }

    // Check if a graph is connected
    int Algorithms::isConnected(ariel::Graph &graph)
    {
        if (graph.getNumVertices() < 1 || graph.getNumEdges() < 1)
        {
            return 0; // Empty graph is not connected
        }

        size_t vertixNum = graph.getNumVertices();

        // Ensure visited array is initialized to false
        vector<bool> visited(vertixNum, false);

        // DFS traversal from an arbitrary vertex
        if (!DFSUtil(graph, 0, visited))
        {
            return 0; // Graph is not connected if a vertex is not reachable
        }

        // Check all vertices in case of disconnected components
        for (size_t i = 0; i < vertixNum; ++i)
        {
            if (!visited[i])
            {
                return 0; // If a vertex is still unvisited, the graph is not connected
            }
        }

        if (graph.isDirected())
        { // If the graph is directed, check the reverse graph as well
            Graph reverseGraph = graph.getTranspose();
            vector<bool> visited((size_t)reverseGraph.getNumVertices(), false);

            // DFS traversal from an arbitrary vertex
            if (!DFSUtil(reverseGraph, 0, visited))
            {
                return 0; // Graph is not connected if a vertex is not reachable
            }

            // Check all vertices in case of disconnected components
            for (size_t i = 0; i < vertixNum; ++i)
            {
                if (!visited[i])
                {
                    return 0; // If a vertex is still unvisited, the graph is not connected
                }
            }
        }

        return 1; // Graph is connected
    }

    // Get the neighbors of a vertex in a graph
    vector<int> Algorithms::getNeighbors(Graph &graph, int vertex)
    {
        size_t VertexNum = graph.getNumVertices(); // Get the number of vertices from the matrix size
        vector<int> neighborsList;

        vector<vector<int>> GraphMat = graph.getAdjacencyMatrix(); // Get the adjacency matrix of the graph

        // Iterate through all vertices in the adjacency matrix row for the given vertex
        unsigned long Vertex = (unsigned long)vertex;
        for (unsigned long neighbor = 0; neighbor < VertexNum; ++neighbor)
        {
            if (GraphMat[Vertex][neighbor] != 0 || GraphMat[neighbor][Vertex] != 0)
            { // Check for an edge (connection)
                neighborsList.push_back(neighbor);
            }
        }

        return neighborsList;
    }

    // Utility function for DFS traversal
    bool Algorithms::DFSUtil(ariel::Graph &graph, size_t Vertex, vector<bool> &visited)
    {
        visited[Vertex] = true; // Mark the current node as visited

        vector<int> neighbors = Algorithms::getNeighbors(graph, static_cast<size_t>(Vertex)); // Call the getNeighbors function
        vector<size_t> castedNeighbors(neighbors.begin(), neighbors.end());                   // Cast the neighbors vector to vector<size_t>

        for (size_t neighbor : castedNeighbors)
        { // Iterate over the elements of the castedNeighbors vector
            if (!visited[neighbor])
            {
                DFSUtil(graph, neighbor, visited);
            }
        }

        return true; // All reachable nodes have been visited
    }

    // Find the shortest path between two vertices in a graph
    string Algorithms::shortestPath(ariel::Graph &graph, int start, int end)
    {
        if (start == end)
        {
            return "0"; // Return "0" if the start and end vertices are the same
        }

        if (graph.getNumVertices() == 0 || graph.getNumEdges() < 1 || start < 0 || end < 0 || start >= graph.getNumVertices() || end >= graph.getNumVertices() || negativeCycle(graph) != "No negative cycle found")
        {
            return "-1"; // Return "-1" if the graph is empty, has less than 1 edge, or the start/end vertices are out of range, or a negative cycle is found
        }

        if (start == end)
        {
            return to_string(start); // Return the start vertex if it is the same as the end vertex
        }

        size_t Strat = (size_t)start;
        size_t End = (size_t)end;
        size_t vertexNum = (size_t)graph.getNumVertices(); // Get the number of vertices in the graph

        // Create a distance vector and initialize all distances as infinite (except source)
        vector<int> dist(vertexNum, numeric_limits<int>::max());
        dist[(unsigned long)Strat] = 0; // Distance from source to itself is 0

        // Create a predecessor vector to store the previous node in the shortest path
        vector<int> prev(vertexNum, -1);

        vector<vector<int>> GraphMat = graph.getAdjacencyMatrix(); // Get the adjacency matrix of the graph

        // Relax all edges V-1 times. If negative cycle is found, return false.
        for (size_t i = 0; i < vertexNum - 1; ++i)
        {
            for (size_t u = 0; u < vertexNum; ++u)
            {
                for (size_t v = 0; v < vertexNum; ++v)
                {
                    // Check if the edge (u, v) exists and if relaxing it improves the distance
                    if (GraphMat[u][v] != 0 && dist[u] + GraphMat[u][v] < dist[v])
                    {
                        dist[v] = dist[u] + GraphMat[u][v];
                        prev[v] = u;
                    }
                }
            }
        }

        if (dist[End] == numeric_limits<int>::max())
        {
            return "-1"; // Return "-1" if there is no path from the start to the end vertex
        }

        stack<int> path;
        size_t current = (size_t)end;

        while (current != -1)
        {
            path.push(current);
            current = (size_t)prev[current];
        }

        // Build the path string from the reconstructed path
        string shortestPath;
        while (!path.empty())
        {
            int vertex = path.top();
            path.pop();
            shortestPath += to_string(vertex) + (path.empty() ? "" : "->");
        }

        return shortestPath;
    }

    // Check if a graph is bipartite and return the partitioning
    string Algorithms::isBipartite(ariel::Graph &graph)
    {
        if (graph.getNumVertices() == 0)
        {
            return "The graph is bipartite: A={}, B={}"; // Empty graph is bipartite
        }

        size_t VertixNum = graph.getNumVertices(); // Get the number of vertices in the graph

        // Create a color vector to store colors assigned to vertices.
        // 0 - Uncolored, 1 - Red, -1 - Blue
        vector<int> color(VertixNum, 0);

        // Get the adjacency matrix of the graph
        vector<vector<int>> GraphMat = graph.getAdjacencyMatrix();

        // BFS to check if any odd-length cycle exists (not bipartite)
        for (size_t u = 0; u < VertixNum; ++u)
        {
            if (color[u] == 0)
            {
                queue<size_t> queue;
                color[u] = 1; // Assign starting color (Red)
                queue.push(u);

                while (!queue.empty())
                {
                    size_t vertex = queue.front();
                    queue.pop();

                    // Check for adjacent vertices
                    for (size_t w = 0; w < VertixNum; ++w)
                    {
                        if (GraphMat[vertex][w] != 0 && GraphMat[w][vertex] != 0)
                        {
                            if (color[w] == 0)
                            {
                                color[w] = -color[vertex]; // Assign opposite color (Blue)
                                queue.push(w);
                            }
                            else if (color[w] == color[vertex])
                            {
                                // Adjacent vertices with the same color (odd-length cycle)
                                return "0"; // Not bipartite
                            }
                        }
                    }
                }
            }
        }

        // Separate vertices based on colors (assuming colors 1 and -1)
        vector<int> group1;
        vector<int> group2;

        for (size_t i = 0; i < VertixNum; ++i)
        {
            if (color[i] == 1)
            {
                group1.push_back(i);
            }
            else
            {
                group2.push_back(i);
            }
        }

        // Construct the output string efficiently (avoid string concatenation)
        stringstream result;
        result << "The graph is bipartite: ";

        if (!group1.empty())
        {
            result << "A={";
            bool firstVertex1 = true;

            for (int vertex : group1)
            {
                if (firstVertex1)
                {
                    result << vertex;
                    firstVertex1 = false;
                }
                else
                {
                    result << ", " << vertex;
                }
            }

            result << "}";
        }

        if (!group1.empty())
        {
            result << ", "; // Add a comma only if both groups have elements
        }

        result << "B={";
        bool firstVertex2 = true;

        for (int vertex : group2)
        {
            if (firstVertex2)
            {
                result << vertex;
                firstVertex2 = false;
            }
            else
            {
                result << ", " << vertex;
            }
        }

        result << "}";

        return result.str();
    }

    // Format the cycle vector as a string
    string formatCycle(const vector<size_t> &cycle)
    {
        string cycleStr = "The cycle is: ";
        for (size_t i = 0; i < cycle.size(); ++i)
        {
            cycleStr += to_string(cycle[i]);
            if (i != cycle.size() - 1)
            {
                cycleStr += "->";
            }
        }
        return cycleStr;
    }

    // Check if a graph contains a negative cycle
    string Algorithms::negativeCycle(Graph &graph)
    {
        if (graph.getNumVertices() < 2 || graph.getNumEdges() < 2)
        {
            return "No negative cycle found"; // Return "No negative cycle found" if the graph is empty or has less than 2 edges/vertices
        }

        vector<vector<int>> GraphMat = graph.getAdjacencyMatrix();
        size_t vertixNum = graph.getNumVertices();

        // Small check if the graph doesn't have negative edges
        bool isnegativeEdges = true;
        for (size_t i = 0; i < GraphMat.size(); i++)
        {
            for (size_t j = 0; j < GraphMat.size(); j++)
            {
                if (GraphMat[i][j] < 0)
                {
                    isnegativeEdges = false;
                    break;
                }
            }
        }

        if (isnegativeEdges)
        {
            return "No negative cycle found"; // Return "No negative cycle found" if the graph doesn't have negative edges
        }

        vector<int> distance(vertixNum, INT_MAX);
        distance[0] = 0;

        for (size_t i = 0; i < vertixNum - 1; i++)
        {
            for (size_t j = 0; j < vertixNum; j++)
            {
                if (GraphMat[i][i] == 0)
                {
                    GraphMat[i][i] = INFINITY; // Change all non-edge values to infinity so the algorithm won't consider them as 0-weight edges
                }
            }
        }

        // Relax all edges V-1 times
        for (int i = 0; i < vertixNum - 1; i++)
        {
            for (size_t j = 0; j < vertixNum; j++)
            {
                for (size_t k = 0; k < vertixNum; k++)
                {
                    if (distance[j] != INT_MAX && (GraphMat[j][k] != 0 || j == k) && distance[j] + GraphMat[j][k] < distance[k])
                    {
                        distance[k] = distance[j] + GraphMat[j][k];
                    }
                }
            }
        }

        // Check for negative weight cycle
        for (size_t i = 0; i < vertixNum; i++)
        {
            for (size_t j = 0; j < vertixNum; j++)
            {
                if (distance[i] != INT_MAX && (GraphMat[i][j] != 0 || i == j) && distance[i] + GraphMat[i][j] < distance[j])
                {
                    return "The graph contains a negative cycle";
                }
            }
        }

        return "No negative cycle found";
    }
}
