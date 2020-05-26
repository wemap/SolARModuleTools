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

#include "SolARMapper.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapper)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARMapper::SolARMapper():ComponentBase(xpcf::toUUID<SolARMapper>())
{
    declareInterface<IMapper>(this);
    declareInjectable<IPointCloudManager>(m_pointCloudManager);
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<IKeyframeRetriever>(m_keyframeRetriever);
}

FrameworkReturnCode SolARMapper::setIdentification(SRef<Identification>& identification)
{
	m_identification = identification;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::getIdentification(SRef<Identification>& identification)
{
	identification = m_identification;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::setCoordinateSystem(SRef<CoordinateSystem>& coordinateSystem)
{
	m_coordinateSystem = coordinateSystem;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::getCoordinateSystem(SRef<CoordinateSystem>& coordinateSystem)
{
	coordinateSystem = m_coordinateSystem;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::setPointCloudManager(SRef<IPointCloudManager>& pointCloudManager)
{
	m_pointCloudManager = pointCloudManager;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::getPointCloudManager(SRef<IPointCloudManager>& pointCloudManager)
{
	pointCloudManager = m_pointCloudManager;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::setKeyframesManager(SRef<IKeyframesManager>& keyframesManager)
{
	m_keyframesManager = keyframesManager;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::getKeyframesManager(SRef<IKeyframesManager>& keyframesManager)
{
	keyframesManager = m_keyframesManager;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::setCovisibilityGraph(SRef<ICovisibilityGraph>& covisibilityGraph)
{
	m_covisibilityGraph = covisibilityGraph;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::getCovisibilityGraph(SRef<ICovisibilityGraph>& covisibilityGraph)
{
	covisibilityGraph = m_covisibilityGraph;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::setKeyframeRetriever(SRef<IKeyframeRetriever>& keyframeRetriever)
{
	m_keyframeRetriever = keyframeRetriever;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::getKeyframeRetriever(SRef<IKeyframeRetriever>& keyframeRetriever)
{
	keyframeRetriever = m_keyframeRetriever;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::saveToFile(std::string file)
{
	LOG_WARNING("Coming soon!");
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::loadFromFile(std::string file)
{
	LOG_WARNING("Coming soon!");
	return FrameworkReturnCode::_SUCCESS;
}

    

}
}
}
