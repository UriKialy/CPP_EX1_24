
namespace ariel
{
    class Algorithms
    {

#include "Algorithms.hpp"
#include "Graph.hpp"
#include <limits.h>
#include <string>
#include <stack>
    public:
        bool isContainsCycle(ariel::Graph g)
        {
            return false;
        }

        int isConnected(ariel::Graph g)
        {
            std::vector<std::vector<int>> G = getAdjacencyMatrix(g);
            for (int i = 0; i < g->djacencyMatrix.size(); i++)
            {
                for (int j = 0; j < g->djacencyMatrix[0].size(); j++)
                {
                    if (g->djacencyMatrix[i][j] == 0 && g->djacencyMatrix[j][i] == 0)
                    {
                        return 0;
                    }
                }
            }
            return 1;
        }

        string shortestPath(ariel::Graph g, int start, int end)
        {
            return BellmanFord( &g,  start,  end);  
        }

        string isBipartite(ariel::Graph g)
        {
            if (g.numVertices == 0)
            {
                return "The graph is bipartite: A={}, B={}.";
            }
            else
                () return isBipartite(const Graph &graph);
        }

        string negativeCycle(ariel::Graph g)
        {
            return "no negative cycle";
        }
// Structure to store a path node with its vertex index and previous node
struct PathNode {
    int vertex;
    int prev;
};

// Function to implement Bellman-Ford algorithm
pair<int, string> BellmanFord(const Graph& graph, int src, int dest) {
    int V = graph.numVertices;

    // Create a distance vector and initialize all distances as infinite (except source)
    vector<int> dist(V, numeric_limits<int>::max());
    dist[src] = 0; // Distance from source to itself is 0

    // Create a predecessor vector to store the previous node in the shortest path
    vector<int> prev(V, -1);

    // Relax all edges V-1 times. If negative cycle is found, return false.
    for (int i = 0; i < V - 1; ++i) {
        for (int u = 0; u < V; ++u) {
            for (int v = 0; v < V; ++v) {
                // Check if the edge (u, v) exists and if relaxing it improves the distance
                if (graph.adjacencyMatrix[u][v] != 0 && dist[u] + graph.adjacencyMatrix[u][v] < dist[v]) {
                    dist[v] = dist[u] + graph.adjacencyMatrix[u][v];
                    prev[v] = u;
                }
            }
        }
    }

    // Check for negative-weight cycles. If there is a cycle, a shorter path
    // will be found even after V-1 relaxations.
    for (int u = 0; u < V; ++u) {
        for (int v = 0; v < V; ++v) {
            if (graph.adjacencyMatrix[u][v] != 0 && dist[u] + graph.adjacencyMatrix[u][v] < dist[v]) {
                throw runtime_error("Graph contains a negative-weight cycle!");
            }
        }
    }

    // Reconstruct the shortest path using predecessor information (if destination is reachable)
    if (dist[dest] == numeric_limits<int>::max()) {
        return make_pair(dist[dest], "Destination not reachable");
    }

    stack<int> path;
    int current = dest;
    while (current != -1) {
        path.push(current);
        current = prev[current];
    }

    // Build the path string from the reconstructed path
    string shortestPath;
    while (!path.empty()) {
        int vertex = path.top();
        path.pop();
        shortestPath += to_string(vertex) + (path.empty() ? "" : " -> ");
    }

    return make_pair(dist[dest], shortestPath);
}



        // Function to check if a graph is bipartite
        pair<vector<int>, vector<int>> isBipartite(const Graph &graph)
        {
            int V = graph.numVertices;

            // Create a color vector to store colors assigned to vertices.
            // 0 - Uncolored, 1 - Red, -1 - Blue
            vector<int> color(V, 0);

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
                            if (graph.adjacencyMatrix[v][w] != 0)
                            {
                                if (color[w] == 0)
                                {
                                    color[w] = -color[v]; // Assign opposite color (Blue)
                                    q.push(w);
                                }
                                else if (color[w] == color[v])
                                {
                                    // Adjacent vertices with same color (odd-length cycle)
                                    cout << "The graph is not bipartite." << endl;
                                    return make_pair(vector<int>(), vector<int>()); // Not bipartite
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
            cout << "The graph is bipartite: A={" << group1 << "} B={" << group2 << "}." << endl;
            return make_pair(group1, group2);
        }
    };
} // namespace ariel
