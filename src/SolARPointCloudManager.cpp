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
	m_pointCloud = xpcf::utils::make_shared<PointCloud>();
}

FrameworkReturnCode SolARPointCloudManager::addPoint(const SRef<CloudPoint> point)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->addPoint(point);
}

FrameworkReturnCode SolARPointCloudManager::addPoints(const std::vector<SRef<CloudPoint>>& points)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->addPoints(points);
}

FrameworkReturnCode SolARPointCloudManager::addPoint(const CloudPoint & point)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->addPoint(point);
}

FrameworkReturnCode SolARPointCloudManager::addPoints(const std::vector<CloudPoint>& points)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->addPoints(points);
}

FrameworkReturnCode SolARPointCloudManager::getPoint(const uint32_t id, SRef<CloudPoint>& point) const
{
	m_pointCloud->acquireLock();
	return m_pointCloud->getPoint(id, point);
}

FrameworkReturnCode SolARPointCloudManager::getPoints(const std::vector<uint32_t>& ids, std::vector<SRef<CloudPoint>>& points) const
{
	m_pointCloud->acquireLock();
	return m_pointCloud->getPoints(ids, points);
}

FrameworkReturnCode SolARPointCloudManager::getAllPoints(std::vector<SRef<CloudPoint>>& points) const
{
	m_pointCloud->acquireLock();
	return m_pointCloud->getAllPoints(points);
}

FrameworkReturnCode SolARPointCloudManager::suppressPoint(const uint32_t id)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->suppressPoint(id);
}

FrameworkReturnCode SolARPointCloudManager::suppressPoints(const std::vector<uint32_t>& ids)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->suppressPoints(ids);
}

DescriptorType SolARPointCloudManager::getDescriptorType() const
{
	m_pointCloud->acquireLock();
	return m_pointCloud->getDescriptorType();
}

FrameworkReturnCode SolARPointCloudManager::setDescriptorType(const DescriptorType & type)
{
	m_pointCloud->acquireLock();
	return m_pointCloud->setDescriptorType(type);
}

bool SolARPointCloudManager::isExistPoint(const uint32_t id) const
{
	m_pointCloud->acquireLock();
	return m_pointCloud->isExistPoint(id);
}

int SolARPointCloudManager::getNbPoints() const
{
	m_pointCloud->acquireLock();
	return m_pointCloud->getNbPoints();
}

FrameworkReturnCode SolARPointCloudManager::saveToFile(const std::string& file) const
{
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);	
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
	ia >> m_pointCloud;
	ifs.close();
	return FrameworkReturnCode::_SUCCESS;
}

const SRef<datastructure::PointCloud>& SolARPointCloudManager::getConstPointCloud() const
{
	return m_pointCloud;
}

std::unique_lock<std::mutex> SolARPointCloudManager::getPointCloud(SRef<datastructure::PointCloud>& pointCloud)
{
	pointCloud = m_pointCloud;
	return m_pointCloud->acquireLock();
}

void SolARPointCloudManager::setPointCloud(const SRef<datastructure::PointCloud> pointCloud)
{
	m_pointCloud = pointCloud;
}

}
}
}
