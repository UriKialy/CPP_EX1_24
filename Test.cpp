#include "doctest.h"
#include "Algorithms.hpp"
#include "Graph.hpp"

using namespace std;

TEST_CASE("Test isConnected")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isConnected(g) == true);

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isConnected(g) == false);
}

TEST_CASE("Test shortestPath")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");
}
TEST_CASE("Test isContainsCycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isContainsCycle(g)=="0");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    string results = ariel::Algorithms::isContainsCycle(g);
     if(ariel::Algorithms::isContainsCycle(g) == "0->1->2->0" || ariel::Algorithms::isContainsCycle(g) == "2->1->0->2"|| 
    ariel::Algorithms::isContainsCycle(g) == "1->0->2->1"){
    }
    else{
        CHECK(false);
    }
}
TEST_CASE("Test isBipartite")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0},
        {1, 0, 1},
        {0, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1}");

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isBipartite(g) == "0");

    vector<vector<int>> graph3 = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph3);
    CHECK(ariel::Algorithms::isBipartite(g) == "0");
}
TEST_CASE("Test invalid graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0},
        {1, 0, 3, 0},
        {2, 3, 0, 4},
        {0, 0, 4, 0},
        {0, 0, 0, 5}};
    CHECK_THROWS(g.loadGraph(graph));
}
TEST_CASE("Test empty graph")
{
    ariel::Graph g;
    vector<vector<int>> graph = {{}};
    CHECK_THROWS(g.loadGraph(graph));
    CHECK(ariel::Algorithms::isBipartite(g) =="The graph is bipartite: A={}, B={}");
    CHECK(ariel::Algorithms::isContainsCycle(g) == "0");
    CHECK(ariel::Algorithms::isConnected(g) == false);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 4) == "-1");
    CHECK(ariel::Algorithms::negativeCycle(g) == "No negative cycle found");
}
TEST_CASE("Test negative cycle")
{
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0, 0},
        {1, 0, 3, 0, 0},
        {2, 3, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::negativeCycle(g) == "No negative cycle found");
    vector<vector<int>> graph2 = {
        {0, 1, 2, 0, 0},
        {1, 0, -6, 0, 0},
        {2, -6, 0, 4, 0},
        {0, 0, 4, 0, 5},
        {0, 0, 0, 5, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::negativeCycle(g) == "The negative cycle is: 1->2->1");
}
TEST_CASE("Test isConnected2") {

  // Test connectivity for a single isolated vertex
  ariel::Graph g;
  vector<vector<int>> graph = {{0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isConnected(g) == 0);

  // Test disconnected graph with multiple components
  vector<vector<int>> graph2 = {
      {0, 0, 0},
      {0, 0, 1},
      {0, 1, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::isConnected(g) == 0);
}

TEST_CASE("Test shortestPath2") {
  // Test non-existent vertices
  ariel::Graph g;
  vector<vector<int>> graph = {{0, 1, 0}, {1, 0, 1}, {0, 1, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::shortestPath(g, 3, 2)=="-1");
  CHECK(ariel::Algorithms::shortestPath(g, -1, 0)=="-1");
  // Test shortest path to itself

  CHECK(ariel::Algorithms::shortestPath(g, 0, 0) == "0");
    vector<vector<int>> graph2 = {{0, -1, 0}, {-1, 0, -2}, {0, -2, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::negativeCycle(g) == "No negative cycle found");
    CHECK_THROWS(ariel::Algorithms::shortestPath(g, 0, 2));

  vector<vector<int>> graph3 = {{0, 1, 0,1}, {1, 0, 1,0}, {0,1,0, 1},{1,0,1,0}};
  g.loadGraph(graph3);
  if((ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2" )|| (ariel::Algorithms::shortestPath(g, 0, 2) == "0->3->2")){
  }
  else{
      CHECK(false);
  }
}

TEST_CASE("Test isContainsCycle2") {
  // Test self-loop
  ariel::Graph g;
  vector<vector<int>> graph = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isContainsCycle(g) == "0");

  // Test disconnected cycles
  vector<vector<int>> graph2 = {
      {0,1,1,0}, {1, 0, 1,0}, {1, 1,0, 0},{0,0,0,1} };
  g.loadGraph(graph2);
  if(ariel::Algorithms::isContainsCycle(g) == "0->1->2->0" || ariel::Algorithms::isContainsCycle(g) == "0->2->1->0"
  ||ariel::Algorithms::isContainsCycle(g) == "2->0->1->2"||ariel::Algorithms::isContainsCycle(g) == "1->2->0->1"
  ||ariel::Algorithms::isContainsCycle(g) == "2->1->0->2"||ariel::Algorithms::isContainsCycle(g) == "1->0->2->1" ){ }
  else{
      CHECK(false);
  }
}

TEST_CASE("Test isBipartite2") {
  // Test odd-length cycle
  ariel::Graph g;
  vector<vector<int>> graph = {{0, 1, 0,0}, {1,0, 0, 0}, {0,0,0, 1}, {0,0,1,0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0, 2}, B={1, 3}");
}

TEST_CASE("Test shortestPath with Multiple Shortest Paths") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 0}};
    g.loadGraph(graph);
    if((ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2") || (ariel::Algorithms::shortestPath(g, 0, 2) == "0->2")){
        }
        else{
            CHECK(false);
        
    }
}

TEST_CASE("Test isBipartite with Single Vertex Graph") {
    ariel::Graph g;
    vector<vector<int>> graph = {{0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "The graph is bipartite: A={0}, B={}");
}

TEST_CASE("Test isBipartite with Non-Bipartite Graph 2") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 0, 0},
        {1, 0, 1, 1},
        {0, 1, 0, 1},
        {0, 1, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isBipartite(g) == "0");
}

TEST_CASE("Test shortestPath with Vertex Outside of Range") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 1},
        {1, 0, 1},
        {1, 1, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 5)=="-1");
}
TEST_CASE("Test negativeCycle with Multiple Negative Cycles") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2},
        {1, 0, -6},
        {2, -6, 0}};
    g.loadGraph(graph);
    string result = ariel::Algorithms::negativeCycle(g);
    if(result == "The negative cycle is: 1->2->1"|| result == "The negative cycle is: 2->1->2"){
    }
    else{
        CHECK(false);
    }
}
TEST_CASE("Test shortestPath with Self-Loop") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 0) == "0");
}
TEST_CASE("Test isContainsCycle with Disconnected Cycles") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2, 0, 0},
        {1, 0, 0, 1, 0},
        {2, 0, 0, 1, 0},
        {0, 1, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph);
    if(ariel::Algorithms::isContainsCycle(g) == "0->1->3->2->0"||ariel::Algorithms::isContainsCycle(g) =="3->2->0->1->3"||
    ariel::Algorithms::isContainsCycle(g) == "1->3->2->0->1"||ariel::Algorithms::isContainsCycle(g) == "2->0->1->3->2"||
    ariel::Algorithms::isContainsCycle(g) == "0->2->3->1->0"||ariel::Algorithms::isContainsCycle(g) == "3->1->0->2->3"||
    ariel::Algorithms::isContainsCycle(g) == "2->3->1->0->2"||ariel::Algorithms::isContainsCycle(g) == "1->0->2->3->1")
    {  }
    else{
        CHECK(false);
    }
}
TEST_CASE("Test negativeCycle with No Negative Cycle") {
    ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 2},
        {1, 0, 3},
        {2, 3, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::negativeCycle(g) == "No negative cycle found");
}
TEST_CASE("Test full graph"){
ariel::Graph g;
    vector<vector<int>> graph = {
        {0, 1, 4},
        {1, 0, 2},
        {4, 2, 0}};
    g.loadGraph(graph);
    CHECK(ariel::Algorithms::isConnected(g) == 1);
    CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2");
        if((ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2") || (ariel::Algorithms::shortestPath(g, 0, 2) == "0->2")){
        }
        else{
            CHECK(false);
        
    }
    CHECK(ariel::Algorithms::isBipartite(g) == "0");
    CHECK(ariel::Algorithms::negativeCycle(g) == "No negative cycle found");
}
