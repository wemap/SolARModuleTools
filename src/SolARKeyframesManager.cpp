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
	m_keyframeCollection = xpcf::utils::make_shared<KeyframeCollection>();
	LOG_DEBUG("SolARKeyframesManager constructor");
}

FrameworkReturnCode SolARKeyframesManager::addKeyframe(const SRef<Keyframe> keyframe)
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->addKeyframe(keyframe);
}

FrameworkReturnCode SolARKeyframesManager::addKeyframe(const Keyframe & keyframe)
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->addKeyframe(keyframe);
}

FrameworkReturnCode SolARKeyframesManager::getKeyframe(const uint32_t id, SRef<Keyframe> & keyframe) const
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->getKeyframe(id, keyframe);
}

FrameworkReturnCode SolARKeyframesManager::getKeyframes(const std::vector<uint32_t>& ids, std::vector<SRef<Keyframe>>& keyframes) const
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->getKeyframes(ids, keyframes);
}

FrameworkReturnCode SolARKeyframesManager::getAllKeyframes(std::vector<SRef<Keyframe>>& keyframes) const
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->getAllKeyframes(keyframes);
}

FrameworkReturnCode SolARKeyframesManager::suppressKeyframe(const uint32_t id)
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->suppressKeyframe(id);
}

DescriptorType SolARKeyframesManager::getDescriptorType() const
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->getDescriptorType();
}

FrameworkReturnCode SolARKeyframesManager::setDescriptorType(const DescriptorType & type)
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->setDescriptorType(type);
}

bool SolARKeyframesManager::isExistKeyframe(const uint32_t id) const
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->isExistKeyframe(id);
}

int SolARKeyframesManager::getNbKeyframes() const
{
	m_keyframeCollection->acquireLock();
	return m_keyframeCollection->getNbKeyframes();
}

FrameworkReturnCode SolARKeyframesManager::saveToFile(const std::string& file) const
{
	std::ofstream ofs(file, std::ios::binary);
	OutputArchive oa(ofs);
	oa << m_keyframeCollection;
	ofs.close();
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARKeyframesManager::loadFromFile(const std::string& file)
{
	std::ifstream ifs(file, std::ios::binary);
	if (!ifs.is_open())
		return FrameworkReturnCode::_ERROR_;
    InputArchive ia(ifs);
	ia >> m_keyframeCollection;
	ifs.close();
	return FrameworkReturnCode::_SUCCESS;
}

const SRef<datastructure::KeyframeCollection>& SolARKeyframesManager::getConstKeyframeCollection() const
{
	return m_keyframeCollection;
}

std::unique_lock<std::mutex> SolARKeyframesManager::getKeyframeCollection(SRef<datastructure::KeyframeCollection>& keyframeCollection)
{
	keyframeCollection = m_keyframeCollection;
	return m_keyframeCollection->acquireLock();
}

void SolARKeyframesManager::setKeyframeCollection(const SRef<datastructure::KeyframeCollection> keyframeCollection)
{
	m_keyframeCollection = keyframeCollection;
}


}
}
}
