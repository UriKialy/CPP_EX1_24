#include "Graph.hpp"
#include <vector>
using namespace std;
namespace ariel
{
    class Graph
    {
    public:
        int numVertices;
        int numEdges;
        std::vector<std::vector<int>> adjacencyMatrix;
        Graph()
        {
            numVertices = 0;
            numEdges = 0;
            this->adjacencyMatrix = {};
        }
        void loadGraph(vector<vector<int>> g)
        {
            // Check if the graph is a square matrix.
            if (g.size() == g[0].size())
            {
                this->numVertices = g.size();
            }
            else
            {
                throw invalid_argument("The graph is not a square matrix");
            }
            this->adjacencyMatrix = g;
            int numedges = 0;
            for (int i = 0; i < numVertices; i++)
            {
                for (int j = 0; j < numVertices; j++)
                {
                    if (adjacencyMatrix[i][j] != 0)
                    {
                        numedges++;
                    }
                }
            }
            this->numEdges = numedges;
        }
        Graph() : numVertices(0)
        {
            adjacencyMatrix = {};
        }
        std::vector<std::vector<int>> getAdjacencyMatrix()
        {
            return this->adjacencyMatrix;
        }
        void printGraph()
        {
            cout << "Graph with " << numVertices << " vertices and " << numEdges << " edges." << endl;
        }
        int getNumVertices()
        {
            return this->numVertices;
        }
        int getNumEdges()
        {
            return this->numEdges;
        }
    };
}