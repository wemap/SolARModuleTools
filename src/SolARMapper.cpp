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

SolARMapper::SolARMapper():ConfigurableBase(xpcf::toUUID<SolARMapper>())
{
    declareInterface<IMapper>(this);
    declareInjectable<IPointCloudManager>(m_pointCloudManager);
    declareInjectable<IKeyframesManager>(m_keyframesManager);
    declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
    declareInjectable<IKeyframeRetriever>(m_keyframeRetriever);
	
	declareProperty("directory", m_directory);
	declareProperty("identificationFileName", m_identificationFileName);
	declareProperty("coordinateFileName", m_coordinateFileName);
	declareProperty("pointCloudManagerFileName", m_pcManagerFileName);
	declareProperty("keyframesManagerFileName", m_kfManagerFileName);
	declareProperty("covisibilityGraphFileName", m_covisGraphFileName);
	declareProperty("keyframeRetrieverFileName", m_kfRetrieverFileName);
	declareProperty("reprojErrorThreshold", m_reprojErrorThres);
	declareProperty("thresConfidence", m_thresConfidence);
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

FrameworkReturnCode SolARMapper::getLocalPointCloud(const SRef<Keyframe>& keyframe, float minWeightNeighbor, std::vector<SRef<CloudPoint>>& localPointCloud)
{	
	std::unique_lock<std::mutex> lock(m_mutex);
	// get neighbor keyframes of the keyframe
	std::vector<uint32_t> neighKeyframesId;
	m_covisibilityGraph->getNeighbors(keyframe->getId(), minWeightNeighbor, neighKeyframesId);
	neighKeyframesId.push_back(keyframe->getId());
	// get all cloud point visibilities from keyframes
	std::map<uint32_t, std::map<uint32_t, uint32_t>> tmpIdxLocalMap;
	for (auto const &it : neighKeyframesId) {
		SRef<Keyframe> keyframe;
		m_keyframesManager->getKeyframe(it, keyframe);
		const std::map<uint32_t, uint32_t> &visibility = keyframe->getVisibility();
		for (auto const &v : visibility)
			tmpIdxLocalMap[v.second][it] = v.first;
	}
	// get local point cloud
	for (auto const &it : tmpIdxLocalMap) {
		SRef<CloudPoint> point;
		if (m_pointCloudManager->getPoint(it.first, point) == FrameworkReturnCode::_SUCCESS)
			localPointCloud.push_back(point);
		else {
			for (auto const &v : it.second) {
				SRef<Keyframe> keyframe;
				m_keyframesManager->getKeyframe(v.first, keyframe);
				keyframe->removeVisibility(v.second, it.first);
			}
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::addCloudPoint(const SRef<CloudPoint>& cloudPoint)
{
	// add point to cloud
	m_pointCloudManager->addPoint(cloudPoint);
	const std::map<uint32_t, uint32_t>& pointVisibility = cloudPoint->getVisibility();
	std::vector<uint32_t> keyframeIds;
	// add visibility to keyframes
	for (auto const &v : pointVisibility) {		
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(v.first, keyframe) == FrameworkReturnCode::_SUCCESS) {
			keyframeIds.push_back(v.first);
			keyframe->addVisibility(v.second, cloudPoint->getId());
		}
	}
	// update covisibility graph
	for (int i = 0; i < keyframeIds.size() - 1; i++)
		for (int j = i + 1; j < keyframeIds.size(); j++)
			m_covisibilityGraph->increaseEdge(keyframeIds[i], keyframeIds[j], 1);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::removeCloudPoint(const SRef<CloudPoint>& cloudPoint)
{	
	const std::map<uint32_t, uint32_t>& pointVisibility = cloudPoint->getVisibility();
	std::vector<uint32_t> keyframeIds;
	// remove visibility from keyframes
	for (auto const &v : pointVisibility) {
		SRef<Keyframe> keyframe;
		if (m_keyframesManager->getKeyframe(v.first, keyframe) == FrameworkReturnCode::_SUCCESS) {
			keyframeIds.push_back(v.first);
			keyframe->removeVisibility(v.second, cloudPoint->getId());
		}
	}
	// update covisibility graph
	for (int i = 0; i < keyframeIds.size() - 1; i++)
		for (int j = i + 1; j < keyframeIds.size(); j++)
			m_covisibilityGraph->decreaseEdge(keyframeIds[i], keyframeIds[j], 1);

	// add point to cloud
	m_pointCloudManager->suppressPoint(cloudPoint->getId());
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::removeKeyframe(const SRef<Keyframe>& keyframe)
{
	const std::map<uint32_t, uint32_t>& keyframeVisibility = keyframe->getVisibility();
	// remove visibility of point cloud
	for (auto const &v : keyframeVisibility) {
		SRef<CloudPoint> point;
		if (m_pointCloudManager->getPoint(v.second, point) == FrameworkReturnCode::_SUCCESS)
			point->removeVisibility(keyframe->getId(), v.first);
	}
	// remove covisibility graph
	m_covisibilityGraph->suppressNode(keyframe->getId());
	// remove keyframe
	m_keyframesManager->suppressKeyframe(keyframe->getId());
	return FrameworkReturnCode::_SUCCESS;
}

void SolARMapper::pruning(const std::vector<SRef<CloudPoint>> &cloudPoints)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	// get cloud points
	std::vector<SRef<CloudPoint>> cloudPointsPruning;
	if (cloudPoints.size() == 0) {
		m_pointCloudManager->getAllPoints(cloudPointsPruning);
	}
	else {
		cloudPointsPruning = cloudPoints;
	}

	// check reprojection error to prune cloud points
	int count(0);
	for (const auto &it : cloudPointsPruning)
		if ((it->getReprojError() > m_reprojErrorThres) || (it->getConfidence() < m_thresConfidence)) {
			this->removeCloudPoint(it);
			count++;
		}

	LOG_INFO("Number pruning cloud points: {}", count);
}

FrameworkReturnCode SolARMapper::saveToFile()
{
	LOG_INFO("Saving the map to file...");
	boost::filesystem::create_directory(boost::filesystem::path(m_directory.c_str()));
	LOG_DEBUG("Save identification");
	std::ofstream ofs_iden(m_directory + "/" + m_identificationFileName, std::ios::binary);
	OutputArchive oa_iden(ofs_iden);
	oa_iden << m_identification;
	ofs_iden.close();
	LOG_DEBUG("Save coordinate system");
	std::ofstream ofs_coor(m_directory + "/" + m_coordinateFileName, std::ios::binary);
	OutputArchive oa_coor(ofs_coor);
	oa_coor << m_coordinateSystem;
	ofs_coor.close();
	LOG_DEBUG("Save point cloud manager");
	if (m_pointCloudManager->saveToFile(m_directory + "/" + m_pcManagerFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_DEBUG("Save keyframes manager");
	if (m_keyframesManager->saveToFile(m_directory + "/" + m_kfManagerFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_DEBUG("Save covisibility graph");
	if (m_covisibilityGraph->saveToFile(m_directory + "/" + m_covisGraphFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_DEBUG("Save keyframe retriever");
	if (m_keyframeRetriever->saveToFile(m_directory + "/" + m_kfRetrieverFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_INFO("Save done!");
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::loadFromFile()
{
	LOG_INFO("Loading the map from file...");
	LOG_DEBUG("Load identification");
	std::ifstream ifs_iden(m_directory + "/" + m_identificationFileName, std::ios::binary);
	if (!ifs_iden.is_open())
		return FrameworkReturnCode::_ERROR_;
	InputArchive ia_iden(ifs_iden);
	ia_iden >> m_identification;
	ifs_iden.close();
	LOG_DEBUG("Load coordinate system");
	std::ifstream ifs_coor(m_directory + "/" + m_coordinateFileName, std::ios::binary);
	if (!ifs_coor.is_open())
		return FrameworkReturnCode::_ERROR_;
	InputArchive ia_coor(ifs_coor);
	ia_coor >> m_coordinateSystem;
	ifs_coor.close();
	LOG_DEBUG("Load point cloud manager");
	if (m_pointCloudManager->loadFromFile(m_directory + "/" + m_pcManagerFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_DEBUG("Load keyframes manager");
	if (m_keyframesManager->loadFromFile(m_directory + "/" + m_kfManagerFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_DEBUG("Load covisibility graph");
	if (m_covisibilityGraph->loadFromFile(m_directory + "/" + m_covisGraphFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_DEBUG("Load keyframe retriever");
	if (m_keyframeRetriever->loadFromFile(m_directory + "/" + m_kfRetrieverFileName) == FrameworkReturnCode::_ERROR_)
		return FrameworkReturnCode::_ERROR_;
	LOG_INFO("Load done!");
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARMapper::set(const SRef<IMapper> &floating_mapper) {
	floating_mapper->getKeyframesManager(m_keyframesManager);
	floating_mapper->getKeyframeRetriever(m_keyframeRetriever);
	floating_mapper->getPointCloudManager(m_pointCloudManager);
	floating_mapper->getCovisibilityGraph(m_covisibilityGraph);

	return FrameworkReturnCode::_SUCCESS;
}


}
}
}
