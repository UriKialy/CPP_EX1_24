#include "Graph.hpp"
#include <vector>
using namespace std;
namespace ariel
{
    int numVertices;
    int numEdges;
    std::vector<std::vector<int>> adjacencyMatrix;
    bool isdirected;
    Graph::Graph()
    {
        numVertices = 0;
        numEdges = 0;
        isdirected=false;
        adjacencyMatrix = {};
    }
 
    void Graph::loadGraph(vector<vector<int>> g)
    {
        if (g.empty())
        {
            numVertices = 0;
            numEdges = 0;
            adjacencyMatrix = {}; // Already empty vector
            isdirected = false;
            return;
        }
        

        // Check if the graph is a square matrix.
        if (g.size() == g[0].size())
        {
            numVertices = g.size();
        }
        else
        {
            throw invalid_argument("The graph is not a square matrix");
        }
        adjacencyMatrix = g; // Load the mstrix to the graph
        int numofedges = 0;
        for (size_t i = 0; i < numVertices; i++)
        {
            for (size_t j = 0; j < numVertices; j++)
            {
                if (adjacencyMatrix[i][j] != 0 && adjacencyMatrix[j][i] != 0)
                {
                    numofedges++;
                }
                if(adjacencyMatrix[i][j]!=adjacencyMatrix[j][i] && adjacencyMatrix[i][j]!=0 && adjacencyMatrix[j][i]!=0)
                {
                    throw invalid_argument("multy-graph is not allowed");
                }
                if(adjacencyMatrix[i][j]!=0 && adjacencyMatrix[j][i]==0 || adjacencyMatrix[j][i]!=0 && adjacencyMatrix[i][j]==0)
                {
                    isdirected=true;
                }
            }
        }
        numEdges = (int)numofedges / 2;
        
    }
    std::vector<std::vector<int>> Graph::getAdjacencyMatrix()
    {
        return adjacencyMatrix;
    }
    void Graph::printGraph()
    {
        cout << "Graph with " << numVertices << " vertices and " << numEdges << " edges." << endl;
        // cout<<"the edges are:"<<endl;
        // for (size_t i = 0; i < numVertices; i++)
        // {
        //     for (size_t j = 0; j < numVertices; j++)
        //     {
        //         if (adjacencyMatrix[i][j] != 0)
        //         {
        //             cout << i << " -> " << j << endl;
        //         }
        //     }
        // }
    }
    int Graph::getNumVertices()
    {
        return this->numVertices;
    }
    int Graph::getNumEdges()
    {
        return this->numEdges;
    }
    bool Graph::isDirected()
    {
        return isdirected;
    }
    Graph Graph::getTranspose(){
        Graph g;
        for (size_t i = 0; i < numVertices; i++)
        {
            for (size_t j = 0; j < numVertices; j++)
            {
                g.adjacencyMatrix[i][j]=adjacencyMatrix[j][i];
            }
        }
       g.loadGraph(adjacencyMatrix);
       g.isdirected=isdirected;
       g.numEdges=numEdges;
       g.numVertices=numVertices; 
        return g;
    }
    
}
