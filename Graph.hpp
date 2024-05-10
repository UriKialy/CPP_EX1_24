#ifndef GRAPH_HPP
#define GRAPH_HPP
#include "doctest.h"
#include <iostream>
#include <vector>
#include <stdexcept>
using namespace std;
namespace ariel
{
    class Graph
    {
        vector<vector<int>> adjacencyMatrix;
        int numVertices;
        int numEdges;
        bool isdirected;
    public:
        Graph();
        //this function will load the graph to the object
        void loadGraph(vector<vector<int>> g);

        //this function will print the graph
        void printGraph();  

        //this function will return the adjacency matrix of the graph
        std::vector<std::vector<int>> getAdjacencyMatrix();

        //this function will return the number of vertices in the graph
        int getNumVertices();

        //this function will return the number of edges in the graph
        int getNumEdges();

        //this function will return true if the graph is directed and false if it is not
        bool isDirected();

        //this func will return the reverse graph
        Graph getTranspose();
    };
}
#endif
