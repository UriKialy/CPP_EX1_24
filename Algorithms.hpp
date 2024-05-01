#ifndef ALGORITHMS_HPP
#define ALGORITHMS_HPP
#include <vector>
#include <iostream>
#include <bits/stdc++.h>
#include "Graph.hpp"
#include <string>
namespace ariel{
 class Algorithms{
  public:
    // this function will return true if there is a cycle in the graph. If there is no cycle, it will return false.
    static bool isContainsCycle(  Graph g);

    // this function will return 1 if the graph is connected and 0 if it is not.
    static int isConnected( Graph g);

    // this function will return the shortest path between two vertices as a string. If there is no path, it will return an empty string.
    static string shortestPath( Graph &g, int start, int end);

    // this function will return a string "The graph is bipartite"+the sets if the graph is bipartite and an empty string if it is not.
    static string isBipartite( Graph g);

    // this function will return the negative cycle in the graph as a string if there is one. If there is no negative cycle, it will return "no negative cycle".
    static string negativeCycle( Graph g);

    // this function will return the MST of the graph as a string.  
    static std::vector<int> getNeighbors(Graph graph,int vertex);

    // Recursive DFS helper function
    static bool DFSUtil(ariel::Graph graph, size_t v, std::vector<bool>& visited); 
    
    // DFS function using recursion and tracking parent for cycle detection
    static bool dfsForCycle(std::vector<std::vector<size_t>>& G, std::vector<bool>& visited, std::vector<int>& parent, size_t curr);
   
};
}
#endif
