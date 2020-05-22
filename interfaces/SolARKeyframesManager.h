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

#ifndef SOLARKEYFRAMESMANAGER_H
#define SOLARKEYFRAMESMANAGER_H

#include "api/storage/IKeyframesManager.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {
/**
 * @class SolARKeyframesStorageSet
 * @brief A storage component to store a persistent set of keyframes, based on a std::set.
 */
class SOLAR_TOOLS_EXPORT_API SolARKeyframesManager : public org::bcom::xpcf::ComponentBase,
        public api::storage::IKeyframesManager {
public:

    /// @brief SolARKeypointsStorageSet default constructor
	SolARKeyframesManager();

    /// @brief SolARKeypointsStorageSet default destructor
    ~SolARKeyframesManager() = default;

	/// @brief This method allow to add a frame to the keyframe manager component
	/// @param[in] frame the frame to add to the set of persistent keyframes
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode addKeyframe(const SRef<Keyframe>& keyframe) override;

	/// @brief This method allow to add a frame to the key frame manager component
	/// @param[in] frame the frame to add to the set of persistent keyframes
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode addKeyframe(const Keyframe &keyframe) override;

	/// @brief This method allows to get a keyframe by its id
	/// @param[in] id of the keyframe to get
	/// @param[out] a keyframe stored in the keyframes manager
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getKeyframe(uint32_t id, SRef<Keyframe>& keyframe) override;

	/// @brief This method allows to get a set of keyframes by their ids
	/// @param[in] a vector of ids of the keyframes to get
	/// @param[out] a vector of keyframes stored in the keyframe manager
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getKeyframes(std::vector<uint32_t> &ids, std::vector<SRef<Keyframe>>& keyframes) override;

	/// @brief This method allows to get all keyframes
	/// @param[out] the set of keyframes
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getAllKeyframes(std::vector<SRef<Keyframe>>& keyframes) override;

	/// @brief This method allow to suppress a keyframe by its id
	/// @param[in] id of the keyframe to suppress
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode suppressKeyframe(uint32_t id) override;

	/// @brief This method allows to get the descriptor type used to extract descriptor for each keyframe
	/// @return Descriptor type
	DescriptorType getDescriptorType() override;

	/// @brief This method allows to set the descriptor type used to extract descriptor for each keyframe
	/// @return Descriptor type
	FrameworkReturnCode setDescriptorType(DescriptorType type) override;

	/// @brief This method allows to know if a keyframe is already stored in the component
	/// @param[in] Id of this keyframe
	/// @return true if exist, else false
	bool isExistKeyframe(uint32_t id) override;

	/// @brief This method allows to get the number of keyframes stored in the point cloud
	/// @return The number of keyframes
	int getNbKeyframes() override;

	/// @brief This method allows to save the keyframes to the external file
	/// @param[out] the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode saveToFile(std::string file) override;

	/// @brief This method allows to load the keyframes from the external file
	/// @param[in] the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile(std::string file) override;

    void unloadComponent () override final;


 private:
	 std::map<uint32_t, SRef<Keyframe>>		m_keyframes;
	 DescriptorType							m_descriptorType;
	 uint32_t								m_id;
	 std::mutex								m_mutex;
};

}
}
}

#endif // SOLARKEYFRAMESMANAGER_H
