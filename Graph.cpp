#include "Graph.hpp"
#include <vector>
using namespace std;
namespace ariel
{
    unsigned long numVertices;
    unsigned long numEdges;
    std::vector<std::vector<int>> adjacencyMatrix;
    Graph::Graph()
    {
        numVertices = 0;
        numEdges = 0;
        adjacencyMatrix = {};
    }
    void Graph::loadGraph(vector<vector<int>> g)
    {
        // Check if the graph is a square matrix.
        if (g.size() == g[0].size())
        {
            numVertices = g.size();
        }
        else
        {
            throw invalid_argument("The graph is not a square matrix");
        }
        adjacencyMatrix = g;// Load the mstrix to the graph 
        int numofedges = 0;
        for (unsigned long i = 0; i < numVertices; i++)
        {
            for (unsigned long j = 0; j < numVertices; j++)
            {
                if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[j][i]!=0) 
                {
                    numofedges++;
                }
            }
        }
        numEdges = (unsigned long )numofedges/2;
    }
    std::vector<std::vector<int>> Graph::getAdjacencyMatrix()
    {
        return adjacencyMatrix;
    }
    void Graph::printGraph()
    {
        cout << "Graph with " << numVertices << " vertices and " << numEdges << " edges." << endl;
    }
    int Graph::getNumVertices()
    {
        return this->numVertices;
    }
    int Graph::getNumEdges()
    {
        return this->numEdges;
    }
}