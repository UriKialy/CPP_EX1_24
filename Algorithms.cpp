#include "Algorithms.hpp"
#include "Graph.hpp"
#include <limits.h>
#include <string>
#include <stack>
namespace ariel
{
    // Structure to store a path node with its vertex index and previous node
    struct PathNode
    {
        int vertex;
        int prev;
    };
    bool Algorithms::isContainsCycle(Graph g)
    {
        if (g.getNumVertices() < 2 || g.getNumEdges() < 2)
        {
            return false; // Empty graph or no edges, no cycle
        }

        size_t numVertices = (size_t)g.getNumVertices();

        // Create a visited array to keep track of visited vertices
        vector<bool> visited(numVertices, false);

        // Create a recursion stack to keep track of vertices in the current DFS path
        stack<int> recursionStack;

        // Function to perform DFS traversal
        function<bool(int)> dfs = [&](size_t vertex)
        {
            // Mark the current vertex as visited
            visited[vertex] = true;

            // Push the current vertex to the recursion stack
            recursionStack.push(vertex);

            // Explore neighbors (consider only upper triangular part for undirected graphs)
            for (size_t neighbor = vertex + 1; neighbor < numVertices; neighbor++)
            {
                if (g.getAdjacencyMatrix()[vertex][neighbor] != 0)
                { // Check for edge
                    if (!visited[neighbor])
                    {
                        // If neighbor is not visited, recur for it
                        if (dfs(neighbor))
                        {
                            return true; // Cycle found
                        }
                    }
                    else if (recursionStack.top() != neighbor)
                    {
                        // If neighbor is visited and not present in current DFS path (back edge), cycle found
                        return true;
                    }
                }
            }

            // Pop the vertex from the recursion stack as we have finished exploring its neighbors
            recursionStack.pop();

            return false; // No cycle found in this DFS path
        };

        // Start DFS from each unvisited vertex
        for (size_t i = 0; i < numVertices; i++)
        {
            if (!visited[i])
            {
                if (dfs(i))
                {
                    return true; // Cycle found in the graph
                }
            }
        }

        // No cycle found
        return false;
    }

    int Algorithms::isConnected(ariel::Graph g)
    {
        if (g.getNumVertices() < 1 || g.getNumEdges() < 1)
        {
            return 0; // Empty graph is not connected
        }
        size_t V = (size_t)g.getNumVertices(); // Number of vertices in the graph

        // Create a visited array to keep track of visited nodes
        std::vector<bool> visited(V, false);

        // DFS traversal from an arbitrary vertex
        if (!DFSUtil(g, 0, visited))
        {
            return 0; // Graph is not connected if a vertex is not reachable
        }

        // Since the graph might be directed, we need to repeat DFS
        // for a vertex from each disconnected component explored in the first pass
        for (size_t i = 0; i < V; ++i)
        {
            if (!visited[i])
            {
                return 0; // If a vertex is still unvisited, the graph is not connected
            }
        }

        return 1; // Graph is connected
    }
    std::vector<int> Algorithms::getNeighbors(Graph g, int vertex)
    {

        size_t V = (size_t)g.getNumVertices(); // Get the number of vertices from the matrix size
        std::vector<int> neighborsList;
        std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        // Iterate through all vertices in the adjacency matrix row for the given vertex
        unsigned long v = (unsigned long)vertex;
        for (unsigned long neighbor = 0; neighbor < V; ++neighbor)
        {
            if (G[v][neighbor] == 1)
            { // Check for an edge (connection)
                neighborsList.push_back(neighbor);
            }
        }
        return neighborsList;
    }
    bool Algorithms::DFSUtil(ariel::Graph graph, size_t v, std::vector<bool> &visited)
    {
        visited[v] = true;                         // Mark the current node as visited
        size_t V = (size_t)graph.getNumVertices(); // Get the number of vertices in the graph
        // Recur for all adjacent vertices
        std::vector<int> neighbors = Algorithms::getNeighbors(graph, static_cast<size_t>(v)); // Call the getNeighbors function
        std::vector<size_t> castedNeighbors(neighbors.begin(), neighbors.end());              // Cast the neighbors vector to std::vector<size_t>
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
        if (g.getNumVertices() == 0 || g.getNumEdges() < 1)
        {
            return "-1";
        }
        size_t s = (size_t)start;
        size_t e = (size_t)end;
        size_t V = (size_t)g.getNumVertices(); // Get the number of vertices in the graph
        // Create a distance vector and initialize all distances as infinite (except source)
        vector<int> dist(V, numeric_limits<int>::max());
        dist[(unsigned long)s] = 0; // Distance from source to itself is 0
        // Create a predecessor vector to store the previous node in the shortest path
        vector<int> prev(V, -1);
        std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        // Relax all edges V-1 times. If negative cycle is found, return false.
        for (size_t i = 0; i < V - 1; ++i)
        {
            for (size_t u = 0; u < V; ++u)
            {
                for (size_t v = 0; v < V; ++v)
                {
                    // Check if the edge (u, v) exists and if relaxing it improves the distance
                    if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                    {
                        dist[v] = dist[u] + G[u][v];
                        prev[v] = u;
                    }
                }
            }
        }
        // Check for negative-weight cycles. If there is a cycle, a shorter path
        // will be found even after V-1 relaxations.
        for (size_t u = 0; u < V; ++u)
        {
            for (size_t v = 0; v < V; ++v)
            {
                if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                {
                    throw runtime_error("Graph contains a negative-weight cycle!");
                }
            }
        }
        // Reconstruct the shortest path using predecessor information (if destination is reachable)
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

    string Algorithms::isBipartite(ariel::Graph g)
    {
        if (g.getNumVertices() == 0)
        {
            return "The graph is bipartite: A={}, B={}"; // Empty graph is bipartite
        }
        else
        {
            size_t V = (size_t)g.getNumVertices();

            // Create a color vector to store colors assigned to vertices.
            // 0 - Uncolored, 1 - Red, -1 - Blue
            vector<int> color(V, 0);

            // Get the adjacency matrix of the graph
            std::vector<std::vector<int>> G = g.getAdjacencyMatrix();

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
                            if (G[v][w] != 0)
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
            std::stringstream result;
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

            if (!group2.empty())
            {
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
            }

            return result.str();
        }

        {
            if (g.getNumVertices() == 0)
            {
                return "The graph is bipartite: A={}, B={}.";
            }
            else
            {
                size_t V = (size_t)g.getNumVertices();
                // Create a color vector to store colors assigned to vertices.
                // 0 - Uncolored, 1 - Red, -1 - Blue
                vector<int> color(V, 0);
                std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
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
                                if (G[v][w] != 0)
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
                std::string result = "The graph is bipartite: A={";
                for (int vertex : group1)
                {
                    result += std::to_string(vertex) + ", ";
                }
                // Remove trailing comma and space from group A
                result.erase(result.length() - 2, 2);

                result += "}, B={";
                for (int vertex : group2)
                {
                    result += std::to_string(vertex) + ", ";
                }
                // Remove trailing comma and space from group B
                result.erase(result.length() - 2, 2);

                result += "}";
                return result;
            }
        }
    }

    /* string Algorithms::negativeCycle(ariel::Graph g)
     {
         if (g.getNumVertices() == 0 || g.getNumEdges() < 1)
         {
             return "No negative cycle found";
         }
         // Add a new vertex with outgoing edges to all existing vertices
         size_t n = (size_t)g.getNumVertices();
         std::vector<std::vector<int>> new_adjacency_matrix(n + 1, std::vector<int>(n + 1, 0));
         for (size_t i = 0; i < n; ++i)
         {
             for (size_t j = 0; j < n; ++j)
             {
                 new_adjacency_matrix[i + 1][j + 1] = g.getAdjacencyMatrix()[i][j];
             }
             new_adjacency_matrix[0][i + 1] = 0; // Add an outgoing edge from the new vertex to all existing vertices
         }

         // Run Bellman-Ford with cycle detection
         std::vector<int> distance(n + 1, INT_MAX);
         distance[0] = 0;
         std::vector<int> predecessor(n + 1, -1);

         for (size_t i = 0; i < n; ++i)
         {
             for (size_t u = 0; u < n + 1; ++u)
             {
                 for (size_t v = 0; v < n + 1; ++v)
                 {
                     if (new_adjacency_matrix[u][v] > 0 && distance[u] + new_adjacency_matrix[u][v] < distance[v])
                     {
                         distance[v] = distance[u] + new_adjacency_matrix[u][v];
                         predecessor[v] = u;
                     }
                 }
             }
         }

         // Check for negative cycle in the last iteration
         for (size_t u = 0; u < n + 1; ++u)
         {
             for (size_t v = 0; v < n + 1; ++v)
             {
                 if (new_adjacency_matrix[u][v] > 0 && distance[u] + new_adjacency_matrix[u][v] < distance[v])
                 {
                     // Negative cycle detected

                     // Trace the cycle using predecessor pointers
                     std::string cycle;
                     size_t v_current = v;
                     while (predecessor[v_current] != -1 && cycle.find(std::to_string(v_current)) == std::string::npos)
                     {
                         cycle += std::to_string(v_current) + " -> ";
                         v_current = (size_t)predecessor[v_current];
                     }
                     cycle += std::to_string(v_current);
                     return "Negative cycle: " + cycle;
                 }
             }
         }

         return "No negative cycle found"; // No negative cycle found
     }*/
    //  second version of negative cycle
    string Algorithms::negativeCycle(ariel::Graph graph)
    {
        if (graph.getNumVertices() == 0 || graph.getNumEdges() < 1)
        {
            return "No negative cycle found";
        }
        size_t n = (size_t)graph.getNumVertices();
        vector<int> distance(n, INT_MAX);
        vector<int> predecessor(n, -1);
        int cycle_start = -1;
        distance.resize(n);
        // Assume the source vertex is 0
        distance[0] = 0;
        std::vector<std::vector<int>> G = graph.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        for (size_t i = 0; i < n; ++i)
        {
            cycle_start = -1;
            for (size_t u = 0; u < n; ++u)
            {
                for (size_t v = 0; v < n; ++v)
                {
                    if (G[u][v] != 0)
                    {
                        int new_distance = distance[u] + G[u][v];
                        if (new_distance < distance[v])
                        {
                            distance[v] = new_distance;
                            predecessor[v] = (int)u;
                            cycle_start = v;
                        }
                    }
                }
            }
        }

        vector<int> cycle;
        if (cycle_start != -1)
        {
            // We found a negative cycle
            // Go n steps back to make sure we are in the cycle
            size_t v = (size_t)cycle_start;
            for (size_t i = 0; i < n; ++i)
            {
                v = (size_t)predecessor[(size_t)v];
            }

            // Add vertices to the cycle
            for (int u = v;; u = predecessor[(size_t)u])
            {
                cycle.push_back(u);
                if (u == v && cycle.size() > 1)
                {
                    break;
                }
            }
            reverse(cycle.begin(), cycle.end());

            // Convert cycle to a string representation
            string cycle_str;
            for (size_t i = 0; i < cycle.size(); ++i)
            {
                cycle_str += to_string(cycle[i]);
                if (i != cycle.size() - 1)
                {
                    cycle_str += "->";
                }
            }
            return "The negative cycle is: " + cycle_str;
        }

        return "No negative cycle found"; // No negative cycle found
    }

    /* string Algorithms::negativeCycle(ariel::Graph g)
     {
         if(g.getNumVertices() ==0 || g.getNumEdges() <1)
         {
             return "No negative cycle found";
         }
         size_t V = (size_t)g.getNumVertices();                    // Get the number of vertices in the graph
         std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
         // Create a distance vector and initialize all distances as infinite
         vector<int> dist(V, numeric_limits<int>::max());
         // Create a predecessor vector to store the previous node in the shortest path
         vector<int> prev(V, -1);
         // Relax all edges V-1 times.
         for (size_t i = 0; i < V - 1; ++i)
         {
             for (size_t u = 0; u < V; ++u)
             {
                 for (size_t v = 0; v < V; ++v)
                 {
                     // Check if the edge (u, v) exists and if relaxing it improves the distance
                     if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                     {
                         dist[v] = dist[u] + G[u][v];
                         prev[v] = u;
                     }
                 }
             }
         }
         // Check for negative-weight cycles. If there is a cycle, a shorter path
         // will be found even after V-1 relaxations.
         for (size_t u = 0; u < V; ++u)
         {
             for (size_t v = 0; v < V; ++v)
             {
                 if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                 {
                     // Negative cycle detected
                     // Use a stack to track the cycle
                     stack<int> cycle;
                     size_t current = (size_t)v;
                     // Find the starting vertex of the cycle (loop)
                     do
                     {
                         cycle.push(current);
                         current = (size_t)prev[current];
                     } while (current != -1 && current != cycle.top());

                     // Build the cycle string if a loop is found (not used here)
                     if (!cycle.empty())
                     {
                         string negativeCycleString;
                         negativeCycleString += to_string(cycle.top());
                         cycle.pop();

                         while (!cycle.empty())
                         {
                             negativeCycleString += " -> " + to_string(cycle.top());
                             cycle.pop();
                         }

                         negativeCycleString += " -> " + to_string(cycle.top());

                         return "Negative cycle found: " + negativeCycleString;
                     }
                 }
             }
         }
         // No negative cycle found
         return "No negative cycle found";
     }*/

    bool Algorithms::isDirected(Graph g)
    {
        if (g.getNumVertices() == 0 || g.getNumEdges() < 1)
        {
            return false;
        }
        size_t V = (size_t)g.getNumVertices();                    // Get the number of vertices in the graph
        std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        for (size_t i = 0; i < V; ++i)
        {
            for (size_t j = 0; j < V; ++j)
            {
                if (G[i][j] != G[j][i])
                {
                    return true; // Directed graph
                }
            }
        }
        return false; // Undirected graph
    }
} // namespace ariel
