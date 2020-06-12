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

#ifndef SOLARPOINTCLOUDMANAGER_H
#define SOLARPOINTCLOUDMANAGER_H

#include "api/storage/IPointCloudManager.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"
#include <core/SerializationDefinitions.h>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {
/**
 * @class SolARPointCloudStorageSet
 * @brief A storage component to store a persistent cloud of 3D points, based on a std::set.
 */
class SOLAR_TOOLS_EXPORT_API SolARPointCloudManager : public org::bcom::xpcf::ComponentBase,
        public api::storage::IPointCloudManager {
public:

	SolARPointCloudManager();
	~SolARPointCloudManager() = default;

	/// @brief This method allow to add a 3D point to the point cloud
	/// @param[in] point the 3D point to add to the persistent point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode addPoint(const SRef<CloudPoint>& point) override;

	/// @brief This method allow to add a vector of 3D points to the point cloud
	/// @param[in] a vector of the 3D points to add to the persistent point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode addPoints(const std::vector<SRef<CloudPoint>>& points) override;

	/// @brief This method allow to add a 3D point to the point cloud
	/// @param[in] point the 3D point to add to the persistent point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode addPoint(const CloudPoint &point) override;

	/// @brief This method allow to add a vector of 3D points to the point cloud
	/// @param[in] a vector of the 3D points to add to the persistent point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode addPoints(const std::vector<CloudPoint> &points) override;

	/// @brief This method allows to get a 3D point stored in the point cloud by its id
	/// @param[in] id of the point to get
	/// @param[out] a 3D point stored in the point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getPoint(uint32_t id, SRef<CloudPoint>& point) override;

	/// @brief This method allows to get a set of 3D points stored in the point cloud by their ids
	/// @param[in] a vector of ids of the points to get
	/// @param[out] a vector of 3D points stored in the point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getPoints(std::vector<uint32_t> &ids, std::vector<SRef<CloudPoint>>& points) override;

	/// @brief This method allows to get all 3D points stored in the point cloud
	/// @param[out] the set of 3D point stored in the point cloud
	/// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode getAllPoints(std::vector<SRef<CloudPoint>>& points) override;

	/// @brief This method allow to suppress a point stored in the point cloud by its id
	/// @param[in] id of the point to suppress
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode suppressPoint(uint32_t id) override;

	/// @brief This method allow to suppress a vector of points stored in the point cloud by their ids
	/// @param[in] the vector of ids of the point to suppress
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode suppressPoints(std::vector<uint32_t> &ids) override;

	/// @brief This method allows to get the descriptor type used to extract descriptor for each cloud point
	/// @return Descriptor type
	DescriptorType getDescriptorType() override;

	/// @brief This method allows to set the descriptor type used to extract descriptor for each cloud point
	/// @return Descriptor type
	FrameworkReturnCode setDescriptorType(DescriptorType type) override;

	/// @brief This method allows to know if a point is already stored in the component
	/// @param[in] Id of this point
	/// @return true if exist, else false
	bool isExistPoint(uint32_t id) override;

	/// @brief This method allows to get the number of points stored in the point cloud
	/// @return The number of points
	int getNbPoints() override;

	/// @brief This method allows to save the point cloud to the external file
	/// @param[out] the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode saveToFile(std::string file) override;

	/// @brief This method allows to load the point cloud from the external file
	/// @param[in] the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile(std::string file) override;

	void unloadComponent () override final;


 private:
	std::map<uint32_t, SRef<CloudPoint>>	m_pointCloud;
	DescriptorType							m_descriptorType;
	uint32_t								m_id;
	std::mutex								m_mutex;
};

}
}
}

#endif // SOLARPOINTCLOUDMANAGER_H
