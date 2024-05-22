#include "Graph.hpp"
#include <vector>
using namespace std;
namespace ariel
{
 // Constructor
    Graph::Graph(): numVertices(0), numEdges(0), isdirected(false), adjacencyMatrix(3, vector<int>(3, 0)) {}
 
    
    // Load the graph from a given matrix
    void Graph::loadGraph(vector<vector<int>> graph)
    {
        // Check if the graph is a square matrix.
        if (graph.size() != graph[0].size())
        {
            throw invalid_argument("The graph is not a square matrix");
        }
        if (graph.empty())
        {
            numVertices = 0;
            numEdges = 0;
            adjacencyMatrix = {}; 
            isdirected = false;
            return;
        }
        adjacencyMatrix = graph; // Load the matrix to the graph
        int numofedges = 0;
        numVertices = (int)graph.size(); // Set the number of vertices
        for (size_t i = 0; i < numVertices; i++)
        {
            for (size_t j = 0; j < numVertices; j++)
            {
                if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[j][i] != 0)
                {
                    numofedges++;
                }
                if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[j][i] == 0 || adjacencyMatrix[j][i] != 0 && adjacencyMatrix[i][j] == 0)
                {
                    isdirected = true;
                }
                adjacencyMatrix[i][i] = 0; // Remove self loops caused by matrix initialization
            }
        }
        if (isdirected)
        {
            numEdges = numofedges; // If the graph is directed, the number of edges is the number of edges in the matrix
        }
        numEdges = (int)numofedges / 2; // Otherwise, the number of edges is half the number of edges in the matrix
    }

    // Get the adjacency matrix of the graph
    vector<vector<int>> Graph::getAdjacencyMatrix()
    {
        if (adjacencyMatrix.empty())
        {
            throw invalid_argument("The graph is empty");
        }
        return adjacencyMatrix;
    }
    void Graph::printGraph() const
    {
        cout << "Graph with " << numVertices << " vertices and " << numEdges << " edges." << endl;
    }
    size_t Graph::getNumVertices()const
    {
        return (size_t)this->numVertices;
    }
    int Graph::getNumEdges()const
    {
        return this->numEdges;
    }
    bool Graph::isDirected()const
    {
        return isdirected;
    }
    Graph Graph::getTranspose(){
        Graph graph;
        for (size_t i = 0; i < numVertices; i++)
        {
            for (size_t j = 0; j < numVertices; j++)
            {
                graph.adjacencyMatrix[i][j]=adjacencyMatrix[j][i];
            }
        }
       graph.loadGraph(adjacencyMatrix);
       graph.isdirected=isdirected;
       graph.numEdges=numEdges;
       graph.numVertices=numVertices; 
        return graph;
    }
}
