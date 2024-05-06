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
    CHECK(ariel::Algorithms::isContainsCycle(g) == false);

    vector<vector<int>> graph2 = {
        {0, 1, 1, 0, 0},
        {1, 0, 1, 0, 0},
        {1, 1, 0, 1, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0}};
    g.loadGraph(graph2);
    CHECK(ariel::Algorithms::isContainsCycle(g) == true);
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
    CHECK(ariel::Algorithms::isContainsCycle(g) == false);
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
  CHECK_THROWS(ariel::Algorithms::shortestPath(g, 3, 2));
  CHECK_THROWS(ariel::Algorithms::shortestPath(g, -1, 0));
  // Test shortest path to itself
  CHECK(ariel::Algorithms::shortestPath(g, 0, 0) == "0");
    vector<vector<int>> graph2 = {{0, -1, 0}, {-1, 0, -2}, {0, -2, 0}};
    g.loadGraph(graph2);
    CHECK_THROWS(ariel::Algorithms::shortestPath(g, 0, 2));

  // Test multiple paths with same weight
  vector<vector<int>> graph3 = {{0, 1, 1}, {1, 0, 1}, {1, 1, 0}};
  g.loadGraph(graph3);
  CHECK(ariel::Algorithms::shortestPath(g, 0, 2) == "0->1->2" );
}

TEST_CASE("Test isContainsCycle2") {
  // Test self-loop
  ariel::Graph g;
  vector<vector<int>> graph = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isContainsCycle(g) == true);

  // Test disconnected cycles
  vector<vector<int>> graph2 = {
      {0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0, 1}, {0, 1, 0, 0}};
  g.loadGraph(graph2);
  CHECK(ariel::Algorithms::isContainsCycle(g) == true);
}

TEST_CASE("Test isBipartite2") {
  // Test odd-length cycle
  ariel::Graph g;
  vector<vector<int>> graph = {{0, 1, 0}, {1, 0, 1}, {0, 1, 0}};
  g.loadGraph(graph);
  CHECK(ariel::Algorithms::isBipartite(g) == "0");

  // Test complete graph
  vector<vector<int>> graph2 = {{1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}, {1, 1, 1, 1}};
  g.loadGraph(graph2);
}
