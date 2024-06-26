#ifndef GRAPH_HPP
#define GRAPH_HPP
#include "doctest.h"
#include <iostream>
#include <vector>
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
        void loadGraph(vector<vector<int>> graph);

        //this function will print the graph
        void printGraph()const;  

        //this function will return the adjacency matrix of the graph
        std::vector<vector<int>> getAdjacencyMatrix();

        //this function will return the number of vertices in the graph
        size_t getNumVertices()const;

        //this function will return the number of edges in the graph
        int getNumEdges()const;

        //this function will return true if the graph is directed and false if it is not
        bool isDirected()const;

        //this func will return the reverse graph
        Graph getTranspose();
    };
}
#endif
