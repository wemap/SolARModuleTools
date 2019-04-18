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

#include "SolARKeyframesStorageSet.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARKeyframesStorageSet);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARKeyframesStorageSet::SolARKeyframesStorageSet():ComponentBase(xpcf::toUUID<SolARKeyframesStorageSet>())
{
   addInterface<api::storage::IKeyframesStorage>(this);
}


FrameworkReturnCode SolARKeyframesStorageSet::AddKeyframe(const SRef<Frame> frame)
{
    m_keyframes.insert(frame);
    return FrameworkReturnCode::_SUCCESS;
}


FrameworkReturnCode SolARKeyframesStorageSet::SuppressKeyframe(const SRef<Frame> frame)
{
   m_keyframes.erase(frame);
   return FrameworkReturnCode::_SUCCESS;
}


FrameworkReturnCode SolARKeyframesStorageSet::GetAllKeyframes (std::vector<SRef<Frame>>& frames)
{
   frames.clear();
   frames.assign(m_keyframes.begin(), m_keyframes.end());
   return FrameworkReturnCode::_SUCCESS;
}


bool SolARKeyframesStorageSet::ExistKeyframe(SRef<Frame> frame)
{
    return (m_keyframes.find(frame) != m_keyframes.end());
}


int SolARKeyframesStorageSet::GetNbKeyframes()
{
    return (int)m_keyframes.size();
}

}
}
}
