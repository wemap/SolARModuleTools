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

#ifndef SOLARKEYFRAMESSTORAGESET_H
#define SOLARKEYFRAMESSTORAGESET_H

#include "api/storage/IKeyframesStorage.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

#include <set>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARKeyframesStorageSet : public org::bcom::xpcf::ComponentBase,
        public api::storage::IKeyframesStorage {
public:

    /// @brief SolARKeypointsStorageSet default constructor
    SolARKeyframesStorageSet();

    /// @brief SolARKeypointsStorageSet default destructor
    ~SolARKeyframesStorageSet() = default;

    /// @brief This method allow to add a frame to the key frame storage component
    /// @param[in] frame the frame to add to the set of persistent keyframes
    /// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode AddKeyframe(const SRef<Frame> frame) override;

    /// @brief This method allow to suppress a frame to the key frame storage component
    /// @param[in] frame the frame to suppress to the set of persistent keyframes
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode SuppressKeyframe(const SRef<Frame> frame) override;

    /// @brief This method allows to get all keyframes stored in the component
    /// @param[out] frames a vector storing all the keyframes
    /// /// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode GetAllKeyframes(std::vector<SRef<Frame>>& frames) override;

    /// @brief This method allows to know if a keyframe is already stored in the component
    /// @return true if exist, else false
    bool ExistKeyframe(SRef<Frame> frame) override;

    /// @brief This method allows to get the number of keyframes stored in the component frame storage component
    /// @return The number of keyframes
    int GetNbKeyframes() override;

    void unloadComponent () override final;


 private:
    std::set<SRef<Frame>> m_keyframes;

};

}
}
}

#endif // SOLARKEYFRAMESSTORAGESET_H
