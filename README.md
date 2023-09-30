# graph-negative-cycles

The project is supported by the article [Algorithmic Alchemy: Exploiting Graph Theory in the Foreign Exchange](https://github.com/optiklab/negative-cycles-in-a-graph/wiki/Algorithmic-Alchemy:-Exploiting-Graph-Theory-in-the-Foreign-Exchange).

It contains examples of running the Bellman-Ford algorithm that allows to find a path between two nodes in the graph that meet two essential criteria:
- Graph nodes contain negative weights 
- Start and End nodes are not part of negative cycles

In other words, it handles the issues that cannot be handled by BFS, DFS, Dijkstra or A-Star, explored in [previous article](https://github.com/optiklab/path-algorithms-in-a-graph/wiki).

## Graph for testing

I've used an idea ot Currencies Arbitrage and Foreign Exchanges as a main use cases for this project. Nodes of the graphs represent currencies and edges represent bidirectional exchanges.

The Bellman-Ford algorithm allows me to find profitable Arbitrage Circles.

## How to run

The project is written in C++ with STL use only.
Project file is built using Visual Studio 2022 and Microsoft Windows. So, you basically need to open the project using VS and press F5.

But if you need to run on linux there shouldn't be any problem to build using g++ or clang.

# Author

Copyright (C) 2023 Anton "optiklab" Yarkov

https://github.com/optiklab/graph-negative-cycles

See LICENSE file in the repo.