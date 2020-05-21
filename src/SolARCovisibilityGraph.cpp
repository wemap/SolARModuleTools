/**
 * @copyright Copyright (c) 2017 B-com http://www.b-com.com/
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "SolARCovisibilityGraph.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARCovisibilityGraph);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARCovisibilityGraph::SolARCovisibilityGraph():ComponentBase(xpcf::toUUID<SolARCovisibilityGraph>())
{
   addInterface<api::storage::ICovisibilityGraph>(this);
}

FrameworkReturnCode SolARCovisibilityGraph::increaseEdge(uint32_t node1_id, uint32_t node2_id, uint32_t weight)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::decreaseEdge(uint32_t node1_id, uint32_t node2_id, uint32_t weight)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::removeEdge(uint32_t node1_id, uint32_t node2_id)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::getEdge(uint32_t node1_id, uint32_t node2_id, uint32_t & weight)
{
	return FrameworkReturnCode::_SUCCESS;
}

bool SolARCovisibilityGraph::isEdge(uint32_t node1_id, uint32_t node2_id)
{
	return false;
}

FrameworkReturnCode SolARCovisibilityGraph::getAllNodes(std::vector<uint32_t>& nodes_id)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::suppressNode(uint32_t node_id)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::getNeighbors(uint32_t node, uint32_t minWeight, std::vector<uint32_t>& neighbors)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::minimalSpanningTree(SRef<ICovisibilityGraph> graph)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::maximalSpanningTree(SRef<ICovisibilityGraph> graph)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::getShortestPath(uint32_t node1_id, uint32_t node2_id, std::vector<uint32_t>& path, uint32_t & totalCost)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::saveToFile(std::string file)
{
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARCovisibilityGraph::loadFromFile(std::string file)
{
	return FrameworkReturnCode::_SUCCESS;
}



}
}
}
