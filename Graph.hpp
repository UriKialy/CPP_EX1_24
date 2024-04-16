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
        void loadGraph(vector<vector<int>> g);
        void printGraph();
    };
}