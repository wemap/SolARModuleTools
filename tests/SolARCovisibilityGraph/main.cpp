// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include <iostream>
#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include <api/storage/ICovisibilityGraph.h>
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

int main(int argc, char* argv[])
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

	if (xpcfComponentManager->load("testSolARCovisibilityGraph_conf.xml") != org::bcom::xpcf::_SUCCESS)
	{
		std::cerr << "Failed to load the configuration file testSolARCovisibilityGraph_conf.xml" << std::endl;
		return -1;
	}
    auto covisibilityGraph = xpcfComponentManager->resolve<SolAR::api::storage::ICovisibilityGraph>();

	// create a covisibility graph
	std::string fileName = "covisibility_graph.txt";
	LOG_INFO("Load the covisibity graph from {}", fileName);
	if (covisibilityGraph->loadFromFile(fileName) == FrameworkReturnCode::_ERROR_) {
		LOG_INFO("This file doesn't exist. Create a new covisibility graph");
		covisibilityGraph->increaseEdge(1, 2, 2);
		covisibilityGraph->increaseEdge(1, 2, 1);
		covisibilityGraph->increaseEdge(1, 3, 6);
		covisibilityGraph->decreaseEdge(1, 3, 1);
		covisibilityGraph->increaseEdge(2, 4, 4);
		covisibilityGraph->increaseEdge(4, 3, 12);
		covisibilityGraph->increaseEdge(4, 5, 9);
		covisibilityGraph->increaseEdge(4, 8, 8);
		covisibilityGraph->increaseEdge(5, 8, 1);
		covisibilityGraph->increaseEdge(5, 6, 4);
		covisibilityGraph->increaseEdge(5, 7, 5);
		covisibilityGraph->increaseEdge(6, 7, 6);
		covisibilityGraph->increaseEdge(8, 7, 20);
		covisibilityGraph->saveToFile(fileName);
	}
	else {
		LOG_INFO("Load done");
	}
	// display initial graph
	covisibilityGraph->display();

	LOG_INFO("Shortest path: ");
	std::vector<uint32_t> path;
	covisibilityGraph->getShortestPath(1, 7, path);
	std::cout << "Total cost: " << path.size()-1 << std::endl;
	std::cout << "Path: " << std::endl;
	for (auto const &it : path)
		std::cout << it << " ";
	std::cout << std::endl;

	LOG_INFO("Maximal spanning tree: " );
	std::vector<std::tuple<uint32_t, uint32_t, float>> edges_weights_max;
	float maxTotalWeights;
	covisibilityGraph->maximalSpanningTree(edges_weights_max, maxTotalWeights);
	std::cout << "Total weights: " << maxTotalWeights << std::endl;
	std::cout << "Weights: " << std::endl;
	for (auto const &it : edges_weights_max)
		std::cout << std::get<0>(it) << " - " << std::get<1>(it) << " : " << std::get<2>(it) << std::endl;

	LOG_INFO("Minimal spanning tree: ");
	std::vector<std::tuple<uint32_t, uint32_t, float>> edges_weights_min;
	float minTotalWeights;
	covisibilityGraph->minimalSpanningTree(edges_weights_min, minTotalWeights);
	std::cout << "Total weights: " << minTotalWeights << std::endl;
	std::cout << "Weights: " << std::endl;
	for (auto const &it : edges_weights_min)
		std::cout << std::get<0>(it) << " - " << std::get<1>(it) << " : " << std::get<2>(it) << std::endl;

	// get all nodes
	std::set<uint32_t> nodes;
	covisibilityGraph->getAllNodes(nodes);
	std::cout << "All nodes: " << std::endl;
	for (auto const &it : nodes)
		std::cout << it << " " << std::endl;
	std::cout << std::endl;

	// get neighbors of 4
	std::vector<uint32_t> neighbors;
	covisibilityGraph->getNeighbors(4, 1, neighbors);
	std::cout << "All neighbors of 4: " << std::endl;
	for (auto const &it : neighbors)
		std::cout << it << " " << std::endl;
	std::cout << std::endl;

	// get an edge
	float weight;
	if (covisibilityGraph->getEdge(1, 3, weight) == FrameworkReturnCode::_ERROR_)
		std::cout << "No exist edge between 1 & 3" << std::endl;
	else
		std::cout << "Edge 1-3: " << weight << std::endl;

	// remove a vertex
	covisibilityGraph->suppressNode(1);
	std::cout << "Graph after remove vertex 1:" << std::endl;
	covisibilityGraph->display();

	// get an edge
	if (covisibilityGraph->getEdge(1, 3, weight) == FrameworkReturnCode::_ERROR_)
		std::cout << "No exist edge between 1 & 3" << std::endl;
	else
		std::cout << "Edge 1-3: " << weight;

	// remove an edge
	covisibilityGraph->removeEdge(5, 7);
	std::cout << "Graph after remove edge 5-7:" << std::endl;
	covisibilityGraph->display();

	// check edge 4-8
	if (covisibilityGraph->isEdge(4, 8))
		std::cout << "Exist edge between 4-8" << std::endl;
	else
		std::cout << "No exist edge between 4-8" << std::endl;

	// check edge 5-7
	if (covisibilityGraph->isEdge(5, 7))
		std::cout << "Exist edge between 5-7" << std::endl;
	else
		std::cout << "No exist edge between 5-7" << std::endl;

    return 0;
}
