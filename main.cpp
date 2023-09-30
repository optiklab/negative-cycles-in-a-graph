// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/graph-negative-cycles
// See LICENSE file in the repo.

#include <iostream>
#include <queue>
#include <list>
#include <iomanip>
#include "pathFindingBase.h";

#define NDEBUG

/// <summary>
/// Bellman-Ford algorithm allows to find path between 2 nodes that:
/// - Contain negative weights
/// - Are not part of negative cycles (i.e. graph do not contain negative cycle).
/// 
/// Method of this class implement various algorithms to check if graph contains negative cycle and if NOT: finds path between 2 nodes by Bellman-Ford algorithm.
/// </summary>
class BellmanFordAlgorithm
{
public:
    vector<double> _shortestPath;
    vector<int> _previousVertex;
    bool _solved = false;

    /// <summary>
    /// Checks if graph contains negative cycle and if NOT: finds path between 2 nodes by Bellman-Ford algorithm.
    /// https://www.youtube.com/watch?v=24HziTZ8_xo
    /// </summary>
    bool ContainsNegativeCycles(Graph& graph, int start)
    {
        int verticesNumber = graph.Nodes.size();

        _shortestPath.resize(verticesNumber, INF);
        _previousVertex.resize(verticesNumber, -1);

        _shortestPath[start] = 0;

        bool updated = false;
        // For each vertex, apply relaxation for all the edges V - 1 times.
        // It's important that we should be able to find optimal solution for not more than V - 1 relaxation steps.
        for (int k = 0; k < verticesNumber - 1; k++)
        {
            updated = false;
            for (int from = 0; from < verticesNumber; from++)
            {
                for (int to = 0; to < verticesNumber; to++)
                {
                    if (_shortestPath[to] > _shortestPath[from] + graph.Matrix[from][to])
                    {
                        _shortestPath[to] = _shortestPath[from] + graph.Matrix[from][to];
                        _previousVertex[to] = from;
                        updated = true;
                    }
                }
            }
            if (!updated) // No changes in paths, means we can finish now.
                break;
        }

        // Run one more relaxation step to detect which nodes are part of a negative cycle. 
        // A negative cycle has occurred if we can find a better path beyond the optimal solution.
        if (updated)
        {
            for (int from = 0; from < verticesNumber; from++)
            {
                for (int to = 0; to < verticesNumber; to++)
                {
                    if (_shortestPath[to] > _shortestPath[from] + graph.Matrix[from][to])
                    {
                        return true;
                    }
                }
            }
        }

        _solved = true;

        return false;
    }

    /// <summary>
    /// Checks if graph contains negative cycle and if NOT: finds path between 2 nodes by Bellman-Ford algorithm.
    /// </summary>
    bool ContainsNegativeCycles_Sedgewick(Graph& graph, int start)
    {
        int n = graph.Nodes.size(); // V

        _shortestPath.resize(n, INF);
        _previousVertex.resize(n, -1);

        _shortestPath[start] = 0;

        bool updated = false;
        for (size_t i = 0; i < n; ++i)
        {
            updated = false;
            for (size_t from = 0; from < n; ++from)
            {
                for (size_t to = 0; to < n; ++to)
                {
                    if (from != start && _previousVertex[from] == -1)
                        continue;

                    if (graph.Matrix[from][to] == INF) // Edge not exists
                    {
                        continue;
                    }

                    int new_distance = _shortestPath[from] + graph.Matrix[from][to];
                    if (_shortestPath[to] > new_distance)
                    {
                        _shortestPath[to] = new_distance;
                        _previousVertex[to] = from;
                        updated = true;
                    }
                }
            }

            if (i == n - 1 && updated)
            {
                return true; // Found negative cycle.
            }
        }

        _solved = true;

        return false;
    }

    /// <summary>
    /// Implementation of Sedgewick Fifo algorithm for finding path in the graph with negatie weights (but not negative cycles).
    /// Do not contain protection against cycles.
    /// </summary>
    void FindPathOnly(Graph& graph, int start)
    {
        int n = graph.Nodes.size(); // V

        _shortestPath.resize(n, INF);
        _previousVertex.resize(n, -1);

        _shortestPath[start] = 0;

        int z = 0;

        queue<int> q;
        q.push(start);
        q.push(n);

        while (!q.empty())
        {
            int from = -1;
            while ((from = _QueueGet(q)) == n)
            {
                if (z++ > n)
                {
                    _solved = true;
                    return;
                }
                q.push(n);
            }

            for (size_t to = 0; to < graph.Nodes.size(); ++to)
            {
                if (graph.Matrix[from][to] == INF) // Edge not exists
                {
                    continue;
                }

                double new_distance = _shortestPath[from] + graph.Matrix[from][to];

                if (_shortestPath[to] > new_distance)
                {
                    _shortestPath[to] = new_distance;
                    q.push(to);
                    _previousVertex[to] = from;
                }
            }
        }
    }

    /// <summary>
    /// Whilliam's Fiset implementation of the Bellman-Ford algorithm:
    /// https://github.com/williamfiset/Algorithms/blob/master/src/main/java/com/williamfiset/algorithms/graphtheory/BellmanFordAdjacencyMatrix.java
    /// https://github.com/williamfiset/Algorithms/blob/master/src/main/java/com/williamfiset/algorithms/graphtheory/BellmanFordEdgeList.java
    /// 
    /// Not only checks if graph contains negative cycle, but also points nodes that are part of the cycle.
    /// </summary>
    bool FindPathsAndNegativeCycles(Graph& graph, int start)
    {
        int verticesNumber = graph.Nodes.size();

        _shortestPath.resize(verticesNumber, INF);
        _previousVertex.resize(verticesNumber, -1);

        _shortestPath[start] = 0;

        // For each vertex, apply relaxation for all the edges to find PATH (if there are no negative cycles).
        for (int k = 0; k < verticesNumber - 1; k++)
        {
            for (int from = 0; from < verticesNumber; from++)
            {
                for (int to = 0; to < verticesNumber; to++)
                {
                    if (graph.Matrix[from][to] == INF) // Edge not exists
                    {
                        continue;
                    }

                    if (_shortestPath[to] > _shortestPath[from] + graph.Matrix[from][to])
                    {
                        _shortestPath[to] = _shortestPath[from] + graph.Matrix[from][to];
                        _previousVertex[to] = from;
                    }
                }
            }
        }

        bool negativeCycles = false;

        // Run algorithm a second time to DETECT which nodes are part
        // of a negative cycle. A negative cycle has occurred if we
        // can find a better path beyond the optimal solution.
        for (int k = 0; k < verticesNumber - 1; k++)
        {
            for (int from = 0; from < verticesNumber; from++)
            {
                for (int to = 0; to < verticesNumber; to++)
                {
                    if (graph.Matrix[from][to] == INF) // Edge not exists
                    {
                        continue;
                    }

                    if (_shortestPath[to] > _shortestPath[from] + graph.Matrix[from][to])
                    {
                        _shortestPath[to] = NEG_INF;
                        _previousVertex[to] = -2;
                        negativeCycles = true;
                    }
                }
            }
        }

        _solved = true;

        return negativeCycles;
    }
    
    vector<int> ReconstructShortestPath(Graph& graph, int start, int finish)
    {
        if (_solved)
        {
            list<int> list;
            if (_shortestPath[finish] == NEG_INF)
            {
                cout << "Path from " << start << " to " << finish << " is : Infinite number of shortest paths (negative cycle)." << endl;
                return {};
            }

            for (int at = finish; _previousVertex[at] != -1 && _previousVertex[at] != -2; at = _previousVertex[at])
            {
                list.push_back(at);
            }
            list.push_back(start);

            std::vector<int> path(list.begin(), list.end());
            reverse(path.begin(), path.end());

            cout << "Path from " << start << " to " << finish << " is : ";
            for (int i = 0; i < path.size(); i++)
            {
                cout << path[i] << "(" << graph.Nodes[path[i]].Name << ") ";
            }
            cout << endl;

            return path;
        }
        else
        {
            cout << "Not solved." << endl;
        }

        return {};
    }

private:
    int _QueueGet(queue<int>& queue)
    {
        int v = queue.front();
        queue.pop();
        return v;
    }
};

void runSimple(Graph& graph, int from)
{
    cout << "///////Simplest alg////////////////////////////////" << endl;
    BellmanFordAlgorithm algo1;
    if (algo1.ContainsNegativeCycles(graph, from))
    {
        cout << "Graph contains negative cycle." << endl;
    }
    for (int to = 0; to < graph.Nodes.size(); to++)
    {
        algo1.ReconstructShortestPath(graph, from, to);
    }
}

void runDetectNegativeCycles(Graph& graph, int from)
{
    cout << "///////Detect cycles with Bellmand Ford from Whilliam Fiset////////////////////////////" << endl;
    BellmanFordAlgorithm algo2;
    if (algo2.FindPathsAndNegativeCycles(graph, from))
    {
        cout << "Graph contains negative cycle." << endl;
    }
    for (int to = 0; to < graph.Nodes.size(); to++)
    {
        algo2.ReconstructShortestPath(graph, from, to);
    }
}

void runSedgewick(Graph& graph, int from)
{
    cout << "///////Sedgewick/////////////////////////////////" << endl;
    BellmanFordAlgorithm algo3;
    if (algo3.ContainsNegativeCycles_Sedgewick(graph, from))
    {
        cout << "Graph contains negative cycle." << endl;
    }
    for (int to = 0; to < graph.Nodes.size(); to++)
    {
        algo3.ReconstructShortestPath(graph, from, to);
    }
}

void runSedgewickFifo(Graph& graph, int from)
{
    cout << "///////Sedgewick Fifo///////////////////////////" << endl;
    BellmanFordAlgorithm algo4;
    algo4.FindPathOnly(graph, from);
    for (int to = 0; to < graph.Nodes.size(); to++)
    {
        algo4.ReconstructShortestPath(graph, from, to);
    }
}

void runAllNoNegativeCyclesTests(Graph& graph, int from)
{
    cout << "///////Simple graph without negative cycles////////////////////////////////////////////////" << endl;
    // Test case taken from: https://www.youtube.com/watch?v=24HziTZ8_xo
    graph.Clear();
    graph.Nodes.push_back({ "USD" });
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" });
    graph.Nodes.push_back({ "GBP" });
    graph.Nodes.push_back({ "CNY" });
    //                  USD  CHF   YEN  GBP   CNY
    graph.Matrix = { { 0.0,  6.0,  7.0, INF,  INF },   // USD
                     { INF,  0.0,  8.0, -4.0, 5.0 },   // CHF
                     { INF,  INF,  0.0, 9.0,  -3.0 },  // YEN
                     { INF,  INF,  INF, 0.0,  7.0 },   // GBP
                     { INF,  -2.0, INF, INF,  0.0 } }; // CNY
    from = 0;
    runSimple(graph, from);
    runDetectNegativeCycles(graph, from);
    runSedgewick(graph, from);
    runSedgewickFifo(graph, from);
    // Result:
    // Path from 0 to 0 is : 0(USD)
    // Path from 0 to 1 is : 0(USD) 2(YEN) 4(CNY) 1(CHF)
    // Path from 0 to 2 is : 0(USD) 2(YEN)
    // Path from 0 to 3 is : 0(USD) 2(YEN) 4(CNY) 1(CHF) 3(GBP)
    // Path from 0 to 4 is : 0(USD) 2(YEN) 4(CNY)

    cout << "///////Sedgewick graph without negative cycles/////////////////////////////////////////////" << endl;
    // Test case taken from: picture 21.26, page 341 
    graph.Clear();
    graph.Nodes.push_back({ "USD" });
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" });
    graph.Nodes.push_back({ "GBP" });
    graph.Nodes.push_back({ "CNY" });
    graph.Nodes.push_back({ "EUR" });
    //                 USD    CHF   YEN   GBP   CNY   EUR
    graph.Matrix = { { 0.0,   0.41, INF,  INF,  INF,  0.29 },  // USD
                     { INF,   0.0,  0.51, INF,  0.32, INF },   // CHF
                     { INF,   INF,  0.0,  0.50, INF,  INF },   // YEN
                     { 0.45,  INF,  INF,  0.0,  INF,  -0.38 }, // GBP
                     { INF,   INF,  0.32, 0.36, 0.0,  INF },   // CNY
                     { INF, -0.29,  INF,  INF,  0.21, 0.0 } }; // EUR
    from = 4;
    runSimple(graph, from);
    runDetectNegativeCycles(graph, from);
    runSedgewick(graph, from);
    runSedgewickFifo(graph, from);
    // Result:
    // Path from 4 to 0 is : 4(CNY) 3(GBP) 0(USD)
    // Path from 4 to 1 is : 4(CNY) 3(GBP) 5(EUR) 1(CHF)
    // Path from 4 to 2 is : 4(CNY) 3(GBP) 5(EUR) 1(CHF) 2(YEN)
    // Path from 4 to 3 is : 4(CNY) 3(GBP)
    // Path from 4 to 4 is : 4(CNY)
    // Path from 4 to 5 is : 4(CNY) 3(GBP) 5(EUR)
}

void runOnNegativeCycles(Graph& graph, int from)
{
    cout << "///////Graph with negative cycle////////////////////////////////////////////////" << endl;
    graph.Clear();
    graph.Nodes.push_back({ "USD" }); // 1 (Index = 0)
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" });
    graph.Nodes.push_back({ "GBP" });
    graph.Nodes.push_back({ "CNY" });
    graph.Nodes.push_back({ "EUR" });
    graph.Nodes.push_back({ "XXX" });
    graph.Nodes.push_back({ "YYY" }); // 8  (Index = 7)
    //                 USD  CHF  YEN  GBP   CNY  EUR  XXX  YYY
    graph.Matrix = { { 0.0, 1.0, INF, INF,  INF, INF, INF, INF },   // USD
                     { INF, 0.0, 1.0, INF,  INF, 4.0, 4.0, INF },   // CHF
                     { INF, INF, 0.0, INF,  1.0, INF, INF, INF },   // YEN
                     { INF, INF, 1.0, 0.0,  INF, INF, INF, INF },   // GBP
                     { INF, INF, INF, -3.0, 0.0, INF, INF, INF },   // CNY
                     { INF, INF, INF, INF,  INF, 0.0, 5.0, 3.0 },   // EUR
                     { INF, INF, INF, INF,  INF, INF, 0.0, 4.0 },   // XXX
                     { INF, INF, INF, INF,  INF, INF, INF, 0.0 } }; // YYY
    from = 0;
    runDetectNegativeCycles(graph, from);
    // Result:
    // Graph contains negative cycle.
    // Path from 0 to 0 is : 0(USD)
    // Path from 0 to 1 is : 0(USD) 1(CHF)
    // Path from 0 to 2 is : Infinite number of shortest paths(negative cycle).
    // Path from 0 to 3 is : Infinite number of shortest paths(negative cycle).
    // Path from 0 to 4 is : Infinite number of shortest paths(negative cycle).
    // Path from 0 to 5 is : 0(USD) 1(CHF) 5(EUR)
    // Path from 0 to 6 is : 0(USD) 1(CHF) 6(XXX)
    // Path from 0 to 7 is : 0(USD) 1(CHF) 5(EUR) 7(YYY)
}

void runArbitrageTests(Graph& graph, int from)
{
    cout << "///////Arbitrage simple test cases from Sedgewick////////////////////////////////////////////" << endl;
    // Test case taken from: page 343 and modified in various ways
    graph.Clear();
    graph.Nodes.push_back({ "USD" }); // 1 (Index = 0)
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" });
    graph.Nodes.push_back({ "GBP" });
    graph.Nodes.push_back({ "CNY" }); // 5  (Index = 4)
    //                 USD       CHF     YEN     GBP   CNY
    //               {{ 1.0,    1.631,  0.669, 0.008, 0.686 }, // USD
    //                { 0.613,  1.0,    0.411, 0.005, 0.421 }, // CHF
    //                { 1.495,  2.436,  1.0,   0.012, 1.027 }, // YEN
    //                { 120.5,  197.4,  80.82, 1.0,   82.91 }, // GBP
    //                { 1.459,  2.376,  0.973, 0.012, 1.0 } }; // CNY
    //
    // LogE(x) table:  USD      CHF      YEN     GBP     CNY
    graph.Matrix = { { 0.0,    0.489,  -0.402, -4.791, -0.378 },   // USD
                     { -0.489, 0.0,    -0.891, -5.278, -0.865 },   // CHF
                     { 0.402,  0.89,   0.0,    -4.391, 0.027  },   // YEN
                     { 4.791,  5.285,  4.392,  0.0,    4.418  },   // GBP
                     { 0.378,  0.865,  -0.027, -4.415, 0.0    } }; // CNY
    from = 0;
    runDetectNegativeCycles(graph, from);
    // Result:
    //    Graph contains negative cycle.
    //    Path from 0 to 0 is: Infinite number of shortest paths (negative cycle).
    //    Path from 0 to 1 is: Infinite number of shortest paths (negative cycle).
    //    Path from 0 to 2 is: Infinite number of shortest paths (negative cycle).
    //    Path from 0 to 3 is: Infinite number of shortest paths (negative cycle).
    //    Path from 0 to 4 is: Infinite number of shortest paths (negative cycle).

    cout << "///////Arbitrage simpler test cases (my modifications)////////////////////////////////////////////" << endl;
    graph.Clear();
    graph.Nodes.push_back({ "USD" }); // 1 (Index = 0)
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" }); // 3 (Index = 2)

    // LogE(x) table:   USD      CHF     YEN
    graph.Matrix = { { 0.0,    0.489,  -0.402 },   // USD
                     { -0.489, 0.0,    -0.891 },   // CHF
                     { 0.402,  0.89,   0.0    } }; // YEN
    from = 0;
    runDetectNegativeCycles(graph, from);
    // Result:
    //    Graph contains negative cycle.
    //    Path from 0 to 0 is: Infinite number of shortest paths (negative cycle).
    //    Path from 0 to 1 is: Infinite number of shortest paths (negative cycle).
    //    Path from 0 to 2 is: Infinite number of shortest paths (negative cycle).

    // LogE(x) table:   USD      CHF     YEN
    graph.Matrix = { { 0.0,    0.490,  -0.402 },    // USD
                     { -0.489, 0.0,    -0.891 },    // CHF
                     { 0.403,  0.892,   0.0    } }; // YEN
    from = 0;
    runDetectNegativeCycles(graph, from); // No negative cycle
    // In every pair ($->Rub) or ($->Y) I made positive conversion bigger, that is mean NO negative cycle anymore. Result:
    // Path from 0 to 0 is : 0(USD)
    // Path from 0 to 1 is : 0(USD) 1(CHF)
    // Path from 0 to 2 is : 0(USD) 2(YEN)

    // LogE(x) table:   USD      CHF     YEN
    graph.Matrix = { { 0.0,    0.490,  -0.402 },    // USD
                     { -0.489, 0.0,    -0.891 },    // CHF
                     { 0.403,  0.891,   0.0    } }; // YEN
    from = 0;
    runDetectNegativeCycles(graph, from); // No negative cycle
    // In every pair ($->Rub) or ($->Y) I made positive conversion bigger, that is mean NO negative cycle anymore. Result:
    // Path from 0 to 0 is : 0(USD)
    // Path from 0 to 1 is : 0(USD) 2(YEN) 1(CHF)
    // Path from 0 to 2 is : 0(USD) 2(YEN)

    cout << "///////Arbitrage on bigger graphs////////////////////////////////////////////" << endl;
    graph.Clear();
    graph.Nodes.push_back({ "USD" }); // 1 (Index = 0)
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" });
    graph.Nodes.push_back({ "GBP" });
    graph.Nodes.push_back({ "CNY" }); // 5  (Index = 4)
    // LogE(x) table:  USD      CHF      YEN     GBP     CNY
    graph.Matrix = { { 0.0,    0.490, -0.402, 0.7,  0.413 },   // USD
                     { -0.489, 0.0,   -0.891, 0.89, 0.360 },   // CHF
                     { 0.403,  0.891,  0.0,   0.91, 0.581 },   // YEN
                     { 0.340,  0.405,  0.607, 0.0,  0.72 },   // GBP
                     { 0.403,  0.350,  0.571, 0.71, 0.0 } };   // CNY
    from = 0;
    runDetectNegativeCycles(graph, from);
    // Path from 0 to 0 is : 0(USD)
    // Path from 0 to 1 is : 0(USD) 2(YEN) 1(CHF)
    // Path from 0 to 2 is : 0(USD) 2(YEN)
    // Path from 0 to 3 is : 0(USD) 2(YEN) 3(GBP)
    // Path from 0 to 4 is : 0(USD) 2(YEN) 4(CNY)

    cout << "///////VERY REAL EXAMPLES////////////////////////////////////////////" << endl;
    graph.Clear();
    graph.Nodes.push_back({ "USD" }); // 1 (Index = 0)
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" }); // 3 (Index = 2)

    // LogE(x) table: USD       CHF      YEN
    graph.Matrix = {{ 0.0,      0.1,     -5.01 },   // USD
                    { -0.09,  	0.0,     -5.1  },   // CHF
                    { 5.0,      5.09,    0.0   } }; // YEN
    from = 0;
    runDetectNegativeCycles(graph, from);
    // Result:
    //    Graph contains negative cycle.

    graph.Clear();
    graph.Nodes.push_back({ "USD" }); // 1 (Index = 0)
    graph.Nodes.push_back({ "CHF" });
    graph.Nodes.push_back({ "YEN" }); // 3 (Index = 2)

    graph.Matrix = { { 0.0,     0.12,    -5.01 },  // USD
                    { -0.09,  	0.0,     -5.1  },   // CHF
                    { 5.02,     5.11,    0.0   } }; // YEN
    from = 0;
    runDetectNegativeCycles(graph, from);
    // Result:
    // Path from 0 to 0 is : 0(USD)
    // Path from 0 to 1 is : 0(USD) 2(YEN) 1(CHF)
    // Path from 0 to 2 is : 0(USD) 2(YEN)
}

int main(int argc, char** argv)
{
    Graph graph;
    int from = 0;

    // Simplest use cases: graphs with NO negative cycles.
    runAllNoNegativeCyclesTests(graph, from);

    // Graphs with negative cycles.
    runOnNegativeCycles(graph, from);

    // Run more real use cases.
    runArbitrageTests(graph, from);

    return 0;
}