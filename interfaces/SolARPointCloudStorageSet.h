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

#ifndef SOLARPOINTCLOUDSTORAGESET_H
#define SOLARPOINTCLOUDSTORAGESET_H

#include "api/storage/IPointCloudStorage.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

#include <set>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARPointCloudStorageSet : public org::bcom::xpcf::ComponentBase,
        public api::storage::IPointCloudStorage {
public:

    SolARPointCloudStorageSet();
   ~SolARPointCloudStorageSet() = default;

    /// @brief This method allow to add a 3D point to the key frame storage component
    /// @param[in] point the 3D point to add to the persistent point cloud
    /// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode AddPoint(const SRef<Point3Df> point) override;

    /// @brief This method allow to suppress a frame to the key frame storage component
    /// @param[in] point the 3D point to suppress to the persistent point cloud
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode SuppressPoint(const SRef<Point3Df> point) override;

    /// @brief This method allows to get all 3D points stored in the point cloud
    /// @param[out] the set of 3D point stored in the point cloud
    /// @return FrameworkReturnCode::_SUCCESS_ if succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode GetAllPoints(std::vector<SRef<Point3Df>>& points) override;

    /// @brief This method allows to know if a keyframe is already stored in the component
    /// @return true if exist, else false
    bool ExistPoint(SRef<Point3Df> point) override;

    /// @brief This method allows to get the number of points stored in the point cloud
    /// @return The number of points
    int GetNbPoints() override;

    void unloadComponent () override final;


 private:
    std::set<SRef<Point3Df>> m_points;

};

}
}
}

#endif // SOLARPOINTCLOUDSTORAGESET_H
