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

    bool Algorithms::isCyclicUtil(size_t ver, vector<bool> &visited, vector<bool> &recStack, vector<size_t> &parent, Graph &graph, vector<size_t> &cycle)
    {
        visited[ver] = true;
        recStack[ver] = true;
        bool isDirected = graph.isDirected();
        size_t vertixs = (size_t)graph.getNumVertices();
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

    string Algorithms::isContainsCycle(Graph &graph)
    {
        if (graph.getNumVertices() < 2 || graph.getNumEdges() < 2)
        {
            return "0"; // Return "0" if the graph is empty or has les than 2 edge\vertex
        }
        size_t vertixs = (size_t)graph.getNumVertices();
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

    int Algorithms::isConnected(ariel::Graph &g)
    {
        if (g.getNumVertices() < 1 || g.getNumEdges() < 1)
        {
            return 0; // Empty graph is not connected
        }
        size_t V = (size_t)g.getNumVertices();

        // Ensure visited array is initialized to false
        vector<bool> visited(V, false);

        // DFS traversal from an arbitrary vertex
        if (!DFSUtil(g, 0, visited))
        {

            return 0; // Graph is not connected if a vertex is not reachable
        }

        // Check all vertices in case of disconnected components
        for (size_t i = 0; i < V; ++i)
        { // Iterate through all vertices
            if (!visited[i])
            {
                return 0; // If a vertex is still unvisited, the graph is not connected
            }
        }
        if (g.isDirected())
        { // If the graph is directed, check the reverse graph as well
            Graph reverseGraph = g.getTranspose();
            vector<bool> visited((size_t)reverseGraph.getNumVertices(), false);
            // DFS traversal from an arbitrary vertex
            if (!DFSUtil(reverseGraph, 0, visited))
            {
                return 0; // Graph is not connected if a vertex is not reachable
            }

            // Check all vertices in case of disconnected components
            for (size_t i = 0; i < V; ++i)
            { // Iterate through all vertices
                if (!visited[i])
                {
                    return 0; // If a vertex is still unvisited, the graph is not connected
                }
            }
        }

        return 1; // Graph is connected
    }

    vector<int> Algorithms::getNeighbors(Graph &g, int vertex)
    {

        size_t V = (size_t)g.getNumVertices(); // Get the number of vertices from the matrix size
        vector<int> neighborsList;

        vector<vector<int>> GraphMat = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        // Iterate through all vertices in the adjacency matrix row for the given vertex
        unsigned long v = (unsigned long)vertex;
        for (unsigned long neighbor = 0; neighbor < V; ++neighbor)
        {
            if (GraphMat[v][neighbor] != 0)
            { // Check for an edge (connection)

                neighborsList.push_back(neighbor);
            }
        }
        return neighborsList;
    }
    bool Algorithms::DFSUtil(ariel::Graph &graph, size_t v, vector<bool> &visited)
    {
        visited[v] = true;                                                               // Mark the current node as visited
        size_t V = (size_t)graph.getNumVertices();                                       // Get the number of vertices in the graph
                                                                                         // Recur for all adjacent vertices
        vector<int> neighbors = Algorithms::getNeighbors(graph, static_cast<size_t>(v)); // Call the getNeighbors function
        vector<size_t> castedNeighbors(neighbors.begin(), neighbors.end());              // Cast the neighbors vector to   vector<size_t>
        for (size_t neighbor : castedNeighbors)
        { // Iterate over the elements of the castedNeighbors vector
            if (!visited[neighbor])
            {
                DFSUtil(graph, neighbor, visited);
            }
        }
        return true; // All reachable nodes have been visited
    }
    string Algorithms::shortestPath(ariel::Graph &g, int start, int end)
    {
        if (g.getNumVertices() == 0 || g.getNumEdges() < 1 || start < 0 || end < 0 || start >= g.getNumVertices() || end >= g.getNumVertices())
        {
            return "-1";
        }
        if (negativeCycle(g) != "No negative cycle found")
        {
            throw runtime_error("Graph contains a negative-weight cycle!");
        }
        if (start == end)
        {
            return to_string(start);
        }
        size_t s = (size_t)start;
        size_t e = (size_t)end;
        size_t V = (size_t)g.getNumVertices(); // Get the number of vertices in the graph
        // Create a distance vector and initialize all distances as infinite (except source)
        vector<int> dist(V, numeric_limits<int>::max());
        dist[(unsigned long)s] = 0; // Distance from source to itself is 0
        // Create a predecessor vector to store the previous node in the shortest path
        vector<int> prev(V, -1);
        vector<vector<int>> GraphMat = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        // Relax all edges V-1 times. If negative cycle is found, return false.
        for (size_t i = 0; i < V - 1; ++i)
        {
            for (size_t u = 0; u < V; ++u)
            {
                for (size_t v = 0; v < V; ++v)
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
        if (dist[e] == numeric_limits<int>::max())
        {
            return "-1";
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

    string Algorithms::isBipartite(ariel::Graph &g)
    {
        if (g.getNumVertices() == 0)
        {
            return "The graph is bipartite: A={}, B={}"; // Empty graph is bipartite
        }
        else
        {
            size_t V = (size_t)g.getNumVertices(); // Get the number of vertices in the graph

            // Create a color vector to store colors assigned to vertices.
            // 0 - Uncolored, 1 - Red, -1 - Blue
            vector<int> color(V, 0);

            // Get the adjacency matrix of the graph
            vector<vector<int>> GraphMat = g.getAdjacencyMatrix();

            // BFS to check if any odd-length cycle exists (not bipartite)
            for (size_t u = 0; u < V; ++u)
            {
                if (color[u] == 0)
                {
                    queue<size_t> q;
                    color[u] = 1; // Assign starting color (Red)
                    q.push(u);
                    while (!q.empty())
                    {
                        size_t v = q.front();
                        q.pop();

                        // Check for adjacent vertices
                        for (size_t w = 0; w < V; ++w)
                        {
                            if (GraphMat[v][w] != 0 && GraphMat[w][v] != 0)
                            {
                                if (color[w] == 0)
                                {
                                    color[w] = -color[v]; // Assign opposite color (Blue)
                                    q.push(w);
                                }
                                else if (color[w] == color[v])
                                {
                                    // Adjacent vertices with same color (odd-length cycle)
                                    return "0"; // Not bipartite
                                }
                            }
                        }
                    }
                }
            }

            // Separate vertices based on colors (assuming colors 1 and -1)
            vector<int> group1, group2;
            for (size_t i = 0; i < V; ++i)
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
                result << ", "; // Add comma only if both groups have elements
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
    }
    string Algorithms::negativeCycle(ariel::Graph &graph)
    {
        if (graph.getNumVertices() == 0 || graph.getNumEdges() < 1)
        {
            return "No negative cycle found";
        }

        size_t vertixNumber = (size_t)graph.getNumVertices();
        vector<int> distance(vertixNumber, INT_MAX);
        vector<int> predecessor(vertixNumber, -1);
        int cycle_start = -1;
        distance[0] = 0;
        vector<vector<int>> GraphMat = graph.getAdjacencyMatrix();

        // Run Bellman-Ford algorithm
        for (size_t i = 0; i < vertixNumber; ++i)
        {
            for (size_t u = 0; u < vertixNumber; ++u)
            {
                for (size_t v = 0; v < vertixNumber; ++v)
                {
                    if (GraphMat[u][v] != 0)
                    {
                        int new_distance = distance[u] + GraphMat[u][v];
                        if (new_distance < distance[v])
                        {
                            distance[v] = new_distance;
                            predecessor[v] = (int)u;
                            if (i == vertixNumber - 1)
                                cycle_start = v; // Found a negative cycle
                        }
                    }
                }
            }
        }
        vector<int> cycle;
        if (cycle_start != -1)
        {
            size_t v = (size_t)cycle_start;
            for (size_t i = 0; i < vertixNumber; ++i)
            {
                v = (size_t)predecessor[(size_t)v];
            }

            for (int u = v;; u = predecessor[(size_t)u])
            {
                cycle.push_back(u);
                if (u == v && cycle.size() > 1)
                {
                    break;
                }
            }
            reverse(cycle.begin(), cycle.end());

            string cycle_str;
            for (size_t i = 0; i < cycle.size(); ++i)
            {
                cycle_str += to_string(cycle[i]);
                if (i != cycle.size() - 1)
                {
                    cycle_str += "->";
                }
            }
            if(cycle_str.size()>1)
            return "The negative cycle is: " + cycle_str;
        }

        return "No negative cycle found";
    }

    // string Algorithms::negativeCycle(ariel::Graph graph)
    // {
    //     if (graph.getNumVertices() == 0 || graph.getNumEdges() < 1)
    //     {
    //         return "No negative cycle found";
    //     }
    //     size_t vertixNum = (size_t)graph.getNumVertices();
    //     vector<int> distance(n, INT_MAX);
    //     vector<int> predecessor(n, -1);
    //     int cycle_start = -1;
    //     distance.resize(n);
    //     // Assume the source vertex is 0
    //     distance[0] = 0;
    //       vector<  vector<int>> G = graph.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
    //     for (size_t i = 0; i < n; ++i)
    //     {
    //         cycle_start = -1;
    //         for (size_t u = 0; u < n; ++u)
    //         {
    //             for (size_t v = 0; v < n; ++v)
    //             {
    //                 if (G[u][v] != 0)
    //                 {
    //                     int new_distance = distance[u] + G[u][v];
    //                     if (new_distance < distance[v])
    //                     {
    //                         distance[v] = new_distance;
    //                         predecessor[v] = (int)u;
    //                         cycle_start = v;
    //                     }
    //                 }
    //             }
    //         }
    //     }

    //     vector<int> cycle;
    //     if (cycle_start != -1)
    //     {
    //         // We found a negative cycle
    //         // Go n steps back to make sure we are in the cycle
    //         size_t v = (size_t)cycle_start;
    //         for (size_t i = 0; i < n; ++i)
    //         {
    //             v = (size_t)predecessor[(size_t)v];
    //         }

    //         // Add vertices to the cycle
    //         for (int u = v;; u = predecessor[(size_t)u])
    //         {
    //             cycle.push_back(u);
    //             if (u == v && cycle.size() > 1)
    //             {
    //                 break;
    //             }
    //         }
    //         reverse(cycle.begin(), cycle.end());

    //         // Convert cycle to a string representation
    //         string cycle_str;
    //         for (size_t i = 0; i < cycle.size(); ++i)
    //         {
    //             cycle_str += to_string(cycle[i]);
    //             if (i != cycle.size() - 1)
    //             {
    //                 cycle_str += "->";
    //             }
    //         }
    //         return "The negative cycle is: " + cycle_str;
    //     }

    //     return "No negative cycle found"; // No negative cycle found
    // }
    // Function to find a negative cycle in the graph

    /*
 void Algorithms::negativeCycle(const Graph& g) {
     vector<vector<int>> graph = g.getg();
     vector<int> dist;
     vector<int> pred;
     // Call bellmond-ford method
     bool no_negative_cycle = bellmanFord(graph, 0, dist, pred);   // Call bellmanFord on first vertex
     if (no_negative_cycle)
     {
         cout << "No negative cycle";
         return;
     }

     int vertex; // This will store a vertex which we found in the negative cycle so we can trace it back
     // Relaxation to find a vertex which is inside the negative cycle
     for (int u = 0; u < dist.size(); u++) {
         for (int v = 0; v < dist.size(); v++) {
             int weight = graph[u][v];
             if (weight != 0 && dist[u] != INF && dist[u] + weight < dist[v]) {
                 pred[v] = u;    // Added for tracing it back
                 vertex = u;
             }
         }
     }
     // Now we will trace it back untill we get back to the same point and then print
     int trace = vertex;
     string path = to_string(vertex);   // Add the trace to the string
     for (int i = 0; i < pred.size(); i++)
     {
         path = to_string(pred[trace])+"->"+path;  // Add another trace to the string
         trace = pred[trace];
         if(trace == vertex){
             cout << path;
             return;
         }
     }
 }*/
} // namespace ariel
