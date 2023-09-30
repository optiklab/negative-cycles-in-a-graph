// Copyright (C) 2023 Anton "optiklab" Yarkov
// https://github.com/optiklab/graph-negative-cycles
// See LICENSE file in the repo.
#pragma once
#ifndef Path_Finding_Base_H
#define Path_Finding_Base_H

#include <vector>
#include <memory>

using namespace std;

const double INF = 1000000.0;
const double NEG_INF = -1000000.0;

class GraphNode
{
public:
	string Name;
};

class Graph
{
public:
	void Clear()
	{
		Matrix.clear();
		Edges.clear();
		Nodes.clear();
	}

	vector<vector<double>> Matrix;
	vector<vector<pair<int, int>>> Edges;
	vector<GraphNode> Nodes;
};

#endif