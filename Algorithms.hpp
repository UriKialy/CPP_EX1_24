#include <vector>
#include <iostream>
#include "Graph.hpp"
#include "Graph.cpp"
#include <bits/stdc++.h>
using namespace ariel;
using namespace std;
namespace ariel{
 class Algorithms
{
    public:
        // this function will return true and print  a cycle in the graph if there is one. if there is no cycle it will return 0
          static bool isContainsCycle(ariel::Graph g);

        // this function will return 1 if the graph is connected and 0 if it is not.
         static  int isConnected(ariel::Graph g);

        // this function will return the shortest path between two vertices. if there is no path it will return 1.
        static  string shortestPath(ariel::Graph g, int start, int end);

        // this function will return a string "The graph is bipartite"+the sets if the graph is bipartite and "0" if it is not.
       static  string isBipartite(ariel::Graph g);

        // this function will return the negative cycle in the graph if there is one. if there is no negative cycle it will return "no negative cycle"
        static string negativeCycle(ariel::Graph g);

};
}