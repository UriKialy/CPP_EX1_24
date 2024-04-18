#include "doctest.h"
#include <iostream>
#include <vector>

using namespace std;
namespace ariel
{
    class Graph
    {
        vector<vector<int>> graph;
    public:
        Graph()
        {
            this->graph = {};
        }
        //this function will load the graph to the object
        void loadGraph(vector<vector<int>> g);

        //this function will print the graph
        void printGraph();

        //this function will return the adjacency matrix of the graph
        std::vector<std::vector<int>> getAdjacencyMatrix(Graph g);
    };
}