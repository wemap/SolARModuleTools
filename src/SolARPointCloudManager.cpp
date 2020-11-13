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

#include "SolARPointCloudManager.h"
#include "xpcf/component/ComponentFactory.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARPointCloudManager);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARPointCloudManager::SolARPointCloudManager():ComponentBase(xpcf::toUUID<SolARPointCloudManager>())
{
	addInterface<api::storage::IPointCloudManager>(this);
	m_id = 0;
}

FrameworkReturnCode SolARPointCloudManager::addPoint(const SRef<CloudPoint>& point)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	point->setId(m_id);	
	m_pointCloud[m_id] = point;
	m_id++;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::addPoints(const std::vector<SRef<CloudPoint>>& points)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto &it : points) {
		it->setId(m_id);
		m_pointCloud[m_id] = it;
		m_id++;
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::addPoint(const CloudPoint & point)
{
	std::unique_lock<std::mutex> lock(m_mutex);
    SRef<CloudPoint> point_ptr = xpcf::utils::make_shared<CloudPoint>(point);
	point_ptr->setId(m_id);
	m_pointCloud[m_id] = point_ptr;
	m_id++;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::addPoints(const std::vector<CloudPoint>& points)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto &it : points) {
        SRef<CloudPoint> point_ptr = xpcf::utils::make_shared<CloudPoint>(it);
		point_ptr->setId(m_id);
		m_pointCloud[m_id] = point_ptr;
		m_id++;
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::getPoint(uint32_t id, SRef<CloudPoint>& point)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	std::map< uint32_t, SRef<CloudPoint>>::iterator pointIt = m_pointCloud.find(id);
	if (pointIt != m_pointCloud.end()) {
		point = pointIt->second;
		return FrameworkReturnCode::_SUCCESS;
	}
	else {
		LOG_DEBUG("Cannot find cloud point with id {} to get", id);
		return FrameworkReturnCode::_ERROR_;
	}
}

FrameworkReturnCode SolARPointCloudManager::getPoints(const std::vector<uint32_t>& ids, std::vector<SRef<CloudPoint>>& points)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto &it : ids) {
		std::map< uint32_t, SRef<CloudPoint>>::iterator pointIt = m_pointCloud.find(it);
		if (pointIt == m_pointCloud.end()) {
			LOG_DEBUG("Cannot find cloud point with id {} to get", it);
			return FrameworkReturnCode::_ERROR_;
		}
		points.push_back(pointIt->second);
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::getAllPoints(std::vector<SRef<CloudPoint>>& points)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto pointIt = m_pointCloud.begin(); pointIt != m_pointCloud.end(); pointIt++)
		points.push_back(pointIt->second);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::suppressPoint(uint32_t id)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	std::map< uint32_t, SRef<CloudPoint>>::iterator pointIt = m_pointCloud.find(id);
	if (pointIt != m_pointCloud.end()) {
		m_pointCloud.erase(pointIt);
		return FrameworkReturnCode::_SUCCESS;
	}
	else {
		LOG_DEBUG("Cannot find cloud point with id {} to suppress", id);
		return FrameworkReturnCode::_ERROR_;
	}
}

FrameworkReturnCode SolARPointCloudManager::suppressPoints(const std::vector<uint32_t>& ids)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto &it : ids) {
		std::map< uint32_t, SRef<CloudPoint>>::iterator pointIt = m_pointCloud.find(it);
		if (pointIt == m_pointCloud.end()) {
			LOG_DEBUG("Cannot find cloud point with id {} to suppress", it);
			return FrameworkReturnCode::_ERROR_;
		}
		m_pointCloud.erase(pointIt);
	}
	return FrameworkReturnCode::_SUCCESS;
}

DescriptorType SolARPointCloudManager::getDescriptorType()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	return m_descriptorType;
}

FrameworkReturnCode SolARPointCloudManager::setDescriptorType(DescriptorType type)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_descriptorType = type;
	return FrameworkReturnCode::_SUCCESS;
}

bool SolARPointCloudManager::isExistPoint(uint32_t id)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	if (m_pointCloud.find(id) != m_pointCloud.end())
		return true;
	else 
		return false;
}

int SolARPointCloudManager::getNbPoints()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	return m_pointCloud.size();
}

FrameworkReturnCode SolARPointCloudManager::saveToFile(const std::string& file)
{
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);
	oa << m_id;
	oa << m_descriptorType;
	oa << m_pointCloud;
	ofs.close();
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudManager::loadFromFile(const std::string& file)
{
	std::ifstream ifs(file, std::ios::binary);
	if (!ifs.is_open())
		return FrameworkReturnCode::_ERROR_;
    InputArchive ia(ifs);
	ia >> m_id;
	ia >> m_descriptorType;
	ia >> m_pointCloud;
	ifs.close();
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
