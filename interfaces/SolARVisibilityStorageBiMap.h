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

#ifndef SOLARVISIBILITYSTORAGEBIMAP_H
#define SOLARVISIBILITYSTORAGEBIMAP_H

#include "api/storage/IVisibilityStorage.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

#include <boost/bimap.hpp>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {
/**
 * @class SolARVisibilityStorageBiMap
 * @brief A storage component to store with persistence the visibility between keypoints and 3D points, and respectively, based on a bimap from boost.
 */
class SOLAR_TOOLS_EXPORT_API SolARVisibilityStorageBiMap : public org::bcom::xpcf::ComponentBase,
        public api::storage::IVisibilityStorage {
public:

    SolARVisibilityStorageBiMap();
   ~SolARVisibilityStorageBiMap() = default;

    /// @brief This method allow to add a visibility between a keypoint and the cooresponding 3D point
    /// @param[in] frame the frame to which the keypoint belongs
    /// @param[in] keypoint the keypoint seeing the 3D point
    /// @param[in] point the 3D point seen from the keypoint
    /// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode AddVisibility(const SRef<Frame> frame, const SRef<Keypoint> keypoint, const SRef<Point3Df> point) override;

    /// @brief This method allow to add visibilities between keypoints of a same frame and the corresponding 3D points
    /// @param[in] frame the frame to which the keypoints belongs
    /// @param[in] keypoints the keypoints seeing the 3D points
    /// @param[in] points the 3D points seen from the keypoints
    /// @return FrameworkReturnCode::_SUCCESS_ if the addition succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode AddVisibility(const SRef<Frame> frame, const std::vector<SRef<Keypoint>>& keypoints, const std::vector<SRef<Point3Df>>& points) override;

    /// @brief This method allow to suppress all visibilities related to a 3D point
    /// @param[in] point the 3D point for which we want to suppress all the related visibilities
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode Suppress3DPoint(const SRef<Point3Df> point) override;

    /// @brief This method allow to suppress all visibilities related to a set of 3D points
    /// @param[in] points the set of 3D points for which we want to suppress all the related visibilities
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode Suppress3DPoints(const std::vector<SRef<Point3Df>>& points) override;

    /// @brief This method allow to suppress all visibilities related to a frame
    /// @param[in] frame the frame for which we want to remove the visibilities related to its keypoints
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode SuppressFrame(const SRef<Frame> frame) override;

    /// @brief This method allow to suppress all visibilities related to a keypoint
    /// @param[in] keypoint the keypoint for which we want to suppress all the related visibilities
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode SuppressKeypoint(const SRef<Keypoint> keypoint) override;

    /// @brief This method allow to suppress all visibilities related to a set of keypoints
    /// @param[in] keypoints the set of keypoints for which we want to suppress all the related visibilities
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode SuppressKeypoints(const std::vector<SRef<Keypoint>>& keypoints) override;

    /// @brief This method allows to get all the 3D points visible from a frame
    /// @param[in] frame the frame for which you want to get the visible 3D points
    /// @param[out] visiblePoints the set of 3D points visible from the frame
    /// @return FrameworkReturnCode::_SUCCESS_ if the access to visible 3D points succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode GetVisible3DPoints(const SRef<Frame> frame, std::vector<SRef<Point3Df>>& visiblePoints) override;

    /// @brief This method allows to get all the 3D points visible from a keypoint
    /// @param[in] frame the frame to which the keypoint belongs
    /// @param[in] keypoint the keypoint for wich we want to obtain the visible 3D point
    /// @param[out] visiblePoint the 3D points visible from the keypoint
    /// @return FrameworkReturnCode::_SUCCESS_ if the access to visible 3D points succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode GetVisible3DPoint(const SRef<Frame> frame, const SRef<Keypoint> keypoint, SRef<Point3Df> visiblePoint) override;

    /// @brief This method allows to get all the keypoints and corresponding keyframes seeing a given 3D point
    /// @param[in] frame the frame for which you want to get the visible 3D points
    /// @param[out] visiblePoints the set of 3D points visible from the frame
    /// @return FrameworkReturnCode::_SUCCESS_ if the access to visible 3D points succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode GetVisible3DPoints(const SRef<Point3Df> point, std::vector<SRef<Frame>>& frames, std::vector<SRef<Keypoint>>& keypoints) override;

    /// @brief This method allows to get all the keypoints and corresponding keyframes seeing a given 3D point
    /// @param[in] frame the frame for which you want to get the visible 3D points
    /// @param[out] visiblePoints the set of 3D points visible from the frame
    /// @return FrameworkReturnCode::_SUCCESS_ if the access to visible 3D points succeed, else FrameworkReturnCode::_ERROR.
     FrameworkReturnCode GetVisible3DPoints(const std::vector<SRef<Point3Df>>& points, std::vector<SRef<Frame>>& frames, std::vector<SRef<Keypoint>>& keypoints) override;

    void unloadComponent () override final;


 private:
    boost::bimap<SRef<Keypoint>, SRef<Point3Df>> m_visibility; // Todo, include keyframes

};

}
}
}

#endif // SOLARVISIBILITYSTORAGEBIMAP_H
