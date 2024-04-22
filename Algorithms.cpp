#include "Algorithms.hpp"
#include "Graph.hpp"
#include <limits.h>
#include <string>
#include <stack>
namespace ariel
{
    class Algorithms
    {
    public:
            // Structure to store a path node with its vertex index and previous node
        struct PathNode
        {
            int vertex;
            int prev;
        };
        std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        int V = g.getNumVertices(); // Get the number of vertices in the graph
        bool isContainsCycle(Graph &g)
        {
            // Create a distance vector and initialize all distances as infinite
            vector<int> dist(V, numeric_limits<int>::max());
            // Create a predecessor vector to store the previous node in the shortest path
            vector<int> prev(V, -1);
            // Relax all edges V-1 times.
            int V = g.getNumEdges();
            // Function to check if a graph contains a positive cycle using DFS
            vector<bool> visited(V, false);
            stack<PathNode> dfsStack; // Stack for DFS traversal
            // Perform DFS for each unvisited vertex
            for (int i = 0; i < V; ++i)
            {
                if (!visited[i])
                {
                    dfsStack.push({i, -1}); // Start with no parent for the first vertex
                    while (!dfsStack.empty())
                    {
                        PathNode curr = dfsStack.top();
                        dfsStack.pop();

                        if (visited[curr.vertex])
                        {
                            // Cycle found if visited and not the parent in DFS tree
                            if (curr.vertex != curr.prev)
                            {
                                string cycleString;
                                cycleString += to_string(curr.vertex);

                                // Backtrack to reconstruct the cycle path
                                PathNode backtrack = curr;
                                while (backtrack.prev != -1)
                                {
                                    cycleString = to_string(backtrack.prev) + " -> " + cycleString;
                                    backtrack = dfsStack.top();
                                    dfsStack.pop();
                                }
                                cycleString = to_string(backtrack.vertex) + " -> " + cycleString;

                                cout << "Positive cycle found: " << cycleString << endl;
                                return true; // Terminate after finding a cycle (optional)
                            }
                        }
                        else
                        {
                            visited[curr.vertex] = true;
                            // Explore unvisited neighbors
                            for (int neighbor : G[curr.vertex])
                            {
                                if (!visited[neighbor])
                                {
                                    dfsStack.push({neighbor, curr.vertex}); // Neighbor becomes child
                                }
                            }
                        }
                    }
                }
            }
            // No cycle found
            cout << "0" << endl;
            return false;
        }
        int isConnected(ariel::Graph &g)
        {
            for (int i = 0; i < G.size(); i++)
            {
                for (int j = 0; j < G[0].size(); j++)
                {
                    if (G[i][j] == 0 && G[j][i] == 0)
                    {
                        return 0;
                    }
                }
            }
            return 1;
        }
        string shortestPath(ariel::Graph &g, int start, int end)
        {
            // Create a distance vector and initialize all distances as infinite (except source)
            vector<int> dist(V, numeric_limits<int>::max());
            dist[start] = 0; // Distance from source to itself is 0
            // Create a predecessor vector to store the previous node in the shortest path
            vector<int> prev(V, -1);
            
            // Relax all edges V-1 times. If negative cycle is found, return false.
            for (int i = 0; i < V - 1; ++i)
            {
                for (int u = 0; u < V; ++u)
                {
                    for (int v = 0; v < V; ++v)
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
            for (int u = 0; u < V; ++u)
            {
                for (int v = 0; v < V; ++v)
                {
                    if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                    {
                        throw runtime_error("Graph contains a negative-weight cycle!");
                    }
                }
            }
            // Reconstruct the shortest path using predecessor information (if destination is reachable)
            if (dist[end] == numeric_limits<int>::max())
            {
                return "-1";
            }

            stack<int> path;
            int current = end;
            while (current != -1)
            {
                path.push(current);
                current = prev[current];
            }

            // Build the path string from the reconstructed path
            string shortestPath;
            while (!path.empty())
            {
                int vertex = path.top();
                path.pop();
                shortestPath += to_string(vertex) + (path.empty() ? "" : " -> ");
            }

            return shortestPath;
        }
        string isBipartite(ariel::Graph &g)
        {
            if (g.getNumVertices() == 0)
            {
                return "The graph is bipartite: A={}, B={}.";
            }
            else
                return isBipartiteHelper(g);
        }
        string negativeCycle(ariel::Graph &g)
        {
            // Create a distance vector and initialize all distances as infinite
            vector<int> dist(V, numeric_limits<int>::max());
            // Create a predecessor vector to store the previous node in the shortest path
            vector<int> prev(V, -1);
            // Relax all edges V-1 times.
            for (int i = 0; i < V - 1; ++i)
            {
                for (int u = 0; u < V; ++u)
                {
                    for (int v = 0; v < V; ++v)
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
            for (int u = 0; u < V; ++u)
            {
                for (int v = 0; v < V; ++v)
                {
                    if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                    {
                        // Negative cycle detected
                        // Use a stack to track the cycle
                        stack<int> cycle;
                        int current = v;
                        // Find the starting vertex of the cycle (loop)
                        do
                        {
                            cycle.push(current);
                            current = prev[current];
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
        }
        // Function to check if a graph is bipartite
        string isBipartiteHelper(Graph &graph)
        {
            int V = graph.getNumVertices();
            // Create a color vector to store colors assigned to vertices.
            // 0 - Uncolored, 1 - Red, -1 - Blue
            vector<int> color(V, 0);
            std::vector<std::vector<int>> G = graph.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
            // BFS to check if any odd-length cycle exists (not bipartite)
            for (int u = 0; u < V; ++u)
            {
                if (color[u] == 0)
                {
                    queue<int> q;
                    color[u] = 1; // Assign starting color (Red)
                    q.push(u);
                    while (!q.empty())
                    {
                        int v = q.front();
                        q.pop();

                        // Check for adjacent vertices
                        for (int w = 0; w < V; ++w)
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
            for (int i = 0; i < V; ++i)
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
            return "The graph is bipartite: A={", (group1), "}, B={", (group2), "}.";
        }
    };
} // namespace ariel
