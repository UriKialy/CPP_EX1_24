#include "Graph.hpp"
#include <vector>
using namespace std;

class Graph
{
private:
    int numVertices;
    std::vector<std::vector<int>> adjacencyMatrix;

public:
    void loadGraph(vector<vector<int>> g)
    {
            if(g.size()==g[o].size()){
                numVertices = g.size();
            }
            else{
                throw invalid_argument("The graph is not a square matrix");
            }
            adjacencyMatrix = g;
                if (g.size() == numVertices && g[0].size() == numVertices)

    }
    Graph(): numVertices(0) {
        adjacencyMatrix = {};
    }
    // Graph(int numVertices) : numVertices(numVertices)
    // {
    //     adjacencyMatrix.resize(numVertices, std::vector<int>(numVertices, 0));
    // }

    void printGraph()
    {
        cout << "Graph with " << numVertices << " vertices and " << numEdges() << " edges." << endl;
    }
};
