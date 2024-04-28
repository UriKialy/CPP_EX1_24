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
bool Algorithms::isContainsCycle(Graph g) {
  std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get adjacency matrix
unsigned long V = (unsigned long)g.getNumVertices();      // Get number of vertices

// Replace stack with two vectors for visited and parent tracking
std::vector<bool> visited(V, false);
std::vector<int> parent(V, -1);  // -1 indicates no parent (root)

// Perform DFS for each unvisited vertex
for (int i = 0; i < V; ++i) {
    if (!visited[(unsigned long)i]) {
       const std::vector<std::vector<int>>& adjacencyMatrix =  g.getAdjacencyMatrix(); // Get adjacency matrix
        if (dfsForCycle((std::vector<std::vector<size_t>>&)adjacencyMatrix, visited, parent, i)) {
            // Cycle found
            return true;
        }
    }
}

  // No cycle found
  return false;
}

bool Algorithms::dfsForCycle(std::vector<std::vector<size_t>>& G, std::vector<bool>& visited, std::vector<int>& parent, int curr) {
  visited[(unsigned long)curr] = true;
// Explore unvisited neighbors
for (unsigned long i = 0; i < G[(unsigned long)curr].size(); ++i) {
    size_t neighbor = G[(unsigned long)curr][i];
    if (!visited[neighbor]) {
        parent[neighbor] = curr;  // Track parent for cycle check
        if (dfsForCycle(G, visited, parent, neighbor)) {
            return true;  // Cycle found in a recursive call
        }
    } else if (neighbor != parent[(unsigned long)curr]) {
        // Cycle found if visited and not the parent in DFS tree
        return true;
    }
}

  return false;  // No cycle found in subtree rooted at curr
}
    int Algorithms::isConnected(ariel::Graph g)
    {
          int V = g.getNumVertices(); // Number of vertices in the graph

  // Create a visited array to keep track of visited nodes
  std::vector<bool> visited((unsigned long)V, false);

  // DFS traversal from an arbitrary vertex
  if (!DFSUtil(g, 0, visited)) {
    return 0; // Graph is not connected if a vertex is not reachable
  }

  // Since the graph might be directed, we need to repeat DFS
  // for a vertex from each disconnected component explored in the first pass
  for (int i = 0; i < V; ++i) {
    if (!visited[(unsigned long)i]) {
      return 0; // If a vertex is still unvisited, the graph is not connected
    }
  }

  return 1; // Graph is connected
}
std::vector<int>  Algorithms::getNeighbors(Graph g,unsigned long vertex)  {
    int V = g.getNumVertices();// Get the number of vertices from the matrix size
    std::vector<int> neighborsList;
    std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
    // Iterate through all vertices in the adjacency matrix row for the given vertex
    for (int neighbor = 0; neighbor < V; ++neighbor) {
        if (G[vertex][(unsigned long)neighbor] == 1) { // Check for an edge (connection)
            neighborsList.push_back(neighbor);
        }
    }

    return neighborsList;
}

bool Algorithms::DFSUtil(ariel::Graph graph, int v, std::vector<bool>& visited) {
    visited[(unsigned long)v] = true; // Mark the current node as visited
    unsigned long V = (unsigned long)graph.getNumVertices(); // Get the number of vertices in the graph
    // Recur for all adjacent vertices
    std::vector<int> neighbors = Algorithms::getNeighbors(graph,(unsigned long) v); // Call the getNeighbors function
    for (int neighbor : neighbors) { // Iterate over the elements of the neighbors vector
        if (!visited[(unsigned long)neighbor]) {
            DFSUtil(graph, neighbor, visited);
        }
    }

    return true; // All reachable nodes have been visited
}
     string Algorithms::shortestPath(ariel::Graph &g, int start, int end)
    {
        unsigned int V = (unsigned int)g.getNumVertices(); // Get the number of vertices in the graph
        // Create a distance vector and initialize all distances as infinite (except source)
        vector<int> dist(V, numeric_limits<int>::max());
        dist[(unsigned long)start] = 0; // Distance from source to itself is 0
        // Create a predecessor vector to store the previous node in the shortest path
        vector<int> prev(V, -1);
        std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        // Relax all edges V-1 times. If negative cycle is found, return false.
        for (unsigned long i = 0; i < V - 1; ++i)
        {
            for (unsigned long u = 0; u < V; ++u)
            {
                for (unsigned long v = 0; v < V; ++v)
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
        for (unsigned long u = 0; u < V; ++u)
        {
            for (unsigned long v = 0; v < V; ++v)
            {
                if (G[u][v] != 0 && dist[u] + G[u][v] < dist[v])
                {
                    throw runtime_error("Graph contains a negative-weight cycle!");
                }
            }
        }
        // Reconstruct the shortest path using predecessor information (if destination is reachable)
        if (dist[(unsigned long)end] == numeric_limits<int>::max())
        {
            return "-1";
        }

        stack<int> path;
        int current = end;
        while (current != -1)
        {
            path.push(current);
            current = prev[(unsigned long)current];
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
     string Algorithms::isBipartite(ariel::Graph g)
    {
        if (g.getNumVertices() == 0)
        {
            return "The graph is bipartite: A={}, B={}.";
        }
        else
        {
            unsigned long V = (unsigned long)g.getNumVertices();
            // Create a color vector to store colors assigned to vertices.
            // 0 - Uncolored, 1 - Red, -1 - Blue
            vector<int> color(V, 0);
            std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
            // BFS to check if any odd-length cycle exists (not bipartite)
            for (unsigned long u = 0; u < V; ++u)
            {
                if (color[u] == 0)
                {
                    queue<int> q;
                    color[u] = 1; // Assign starting color (Red)
                    q.push(u);
                    while (!q.empty())
                    {
                        unsigned long v = (unsigned long)q.front();
                        q.pop();

                        // Check for adjacent vertices
                        for (unsigned long w = 0; w < V; ++w)
                        {
                            if ((unsigned long)G[v][w] != 0)
                            {
                                if ((unsigned long)color[w] == 0)
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
            for (unsigned long i = 0; i < V; ++i)
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

            result += "}.";
            return result;
        }
    }
      string Algorithms::negativeCycle(ariel::Graph g)
    {
        unsigned int V = (unsigned int)g.getNumVertices();        // Get the number of vertices in the graph
        std::vector<std::vector<int>> G = g.getAdjacencyMatrix(); // Get the adjacency matrix of the graph
        // Create a distance vector and initialize all distances as infinite
        vector<int> dist(V, numeric_limits<int>::max());
        // Create a predecessor vector to store the previous node in the shortest path
        vector<int> prev(V, -1);
        // Relax all edges V-1 times.
        for (unsigned long i = 0; i < V - 1; ++i)
        {
            for (unsigned long u = 0; u < V; ++u)
            {
                for (unsigned long v = 0; v < V; ++v)
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
        for (unsigned long u = 0; u < V; ++u)
        {
            for (unsigned long v = 0; v < V; ++v)
            {
                if ((unsigned long)G[u][v] != 0 && (unsigned long)dist[u] + (unsigned long)G[u][v] < (unsigned long)dist[v])
                {
                    // Negative cycle detected
                    // Use a stack to track the cycle
                    stack<int> cycle;
                    unsigned long current = v;
                    // Find the starting vertex of the cycle (loop)
                    do
                    {
                        cycle.push(current);
                        current = (unsigned long)prev[current];
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
} // namespace ariel
