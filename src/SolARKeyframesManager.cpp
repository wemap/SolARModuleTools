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

#include "SolARKeyframesManager.h"
#include "xpcf/component/ComponentFactory.h"
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARKeyframesManager);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARKeyframesManager::SolARKeyframesManager():ComponentBase(xpcf::toUUID<SolARKeyframesManager>())
{
	addInterface<api::storage::IKeyframesManager>(this);
	m_id = 0;
}

FrameworkReturnCode SolARKeyframesManager::addKeyframe(const SRef<Keyframe>& keyframe)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	keyframe->setId(m_id);
	m_keyframes[m_id] = keyframe;
	m_id++;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARKeyframesManager::addKeyframe(const Keyframe & keyframe)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    SRef<Keyframe> keyframe_ptr = xpcf::utils::make_shared<Keyframe>(keyframe);
	keyframe_ptr->setId(m_id);
	m_keyframes[m_id] = keyframe_ptr;
	m_id++;
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARKeyframesManager::getKeyframe(uint32_t id, SRef<Keyframe>& keyframe)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	std::map< uint32_t, SRef<Keyframe>>::iterator keyframeIt = m_keyframes.find(id);
	if (keyframeIt != m_keyframes.end()) {
		keyframe = keyframeIt->second;
		return FrameworkReturnCode::_SUCCESS;
	}
	else {
		LOG_ERROR("Cannot find keyframe with id {} to get", id);
		return FrameworkReturnCode::_ERROR_;
	}
}

FrameworkReturnCode SolARKeyframesManager::getKeyframes(std::vector<uint32_t>& ids, std::vector<SRef<Keyframe>>& keyframes)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto &it : ids) {
		std::map< uint32_t, SRef<Keyframe>>::iterator keyframeIt = m_keyframes.find(it);
		if (keyframeIt == m_keyframes.end()) {
			LOG_ERROR("Cannot find keyframe with id {} to get", it);
			return FrameworkReturnCode::_ERROR_;
		}
		keyframes.push_back(keyframeIt->second);
	}
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARKeyframesManager::getAllKeyframes(std::vector<SRef<Keyframe>>& keyframes)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	for (auto keyframeIt = m_keyframes.begin(); keyframeIt != m_keyframes.end(); keyframeIt++)
		keyframes.push_back(keyframeIt->second);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARKeyframesManager::suppressKeyframe(uint32_t id)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	std::map< uint32_t, SRef<Keyframe>>::iterator keyframeIt = m_keyframes.find(id);
	if (keyframeIt != m_keyframes.end()) {
		m_keyframes.erase(keyframeIt);
		return FrameworkReturnCode::_SUCCESS;
	}
	else {
		LOG_ERROR("Cannot find keyframe with id {} to suppress", id);
		return FrameworkReturnCode::_ERROR_;
	}
}

DescriptorType SolARKeyframesManager::getDescriptorType()
{
	std::unique_lock<std::mutex> lock(m_mutex);
	return m_descriptorType;
}

FrameworkReturnCode SolARKeyframesManager::setDescriptorType(DescriptorType type)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	m_descriptorType = type;
	return FrameworkReturnCode::_SUCCESS;
}

bool SolARKeyframesManager::isExistKeyframe(uint32_t id)
{
	std::unique_lock<std::mutex> lock(m_mutex);
	if (m_keyframes.find(id) != m_keyframes.end())
		return true;
	else
		return false;
}

int SolARKeyframesManager::getNbKeyframes()
{
	std::unique_lock<std::mutex> lock(m_mutex);
    return static_cast<int>(m_keyframes.size());
}

FrameworkReturnCode SolARKeyframesManager::saveToFile(std::string file)
{
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);
	oa << m_id;
	oa << m_descriptorType;
	oa << m_keyframes;
	ofs.close();
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARKeyframesManager::loadFromFile(std::string file)
{
	std::ifstream ifs(file, std::ios::binary);
	if (!ifs.is_open())
		return FrameworkReturnCode::_ERROR_;
    InputArchive ia(ifs);
	ia >> m_id;
	ia >> m_descriptorType;
	ia >> m_keyframes;
	ifs.close();
	return FrameworkReturnCode::_SUCCESS;
}



}
}
}
