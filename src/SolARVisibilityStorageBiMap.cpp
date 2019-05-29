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

#include "SolARVisibilityStorageBiMap.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARVisibilityStorageBiMap);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARVisibilityStorageBiMap::SolARVisibilityStorageBiMap():ComponentBase(xpcf::toUUID<SolARVisibilityStorageBiMap>())
{
   addInterface<api::storage::IVisibilityStorage>(this);
}


FrameworkReturnCode SolARVisibilityStorageBiMap::AddVisibility(const SRef<Frame> frame, const SRef<Keypoint> keypoint, const SRef<Point3Df> point)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::AddVisibility(const SRef<Frame> frame, const std::vector<SRef<Keypoint>>& keypoints, const std::vector<SRef<Point3Df>>& points)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::Suppress3DPoint(const SRef<Point3Df> point)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::Suppress3DPoints(const std::vector<SRef<Point3Df>>& points)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::SuppressFrame(const SRef<Frame> frame)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::SuppressKeypoint(const SRef<Keypoint> keypoint)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::SuppressKeypoints(const std::vector<SRef<Keypoint>>& keypoints)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::GetVisible3DPoints(const SRef<Frame> frame, std::vector<SRef<Keypoint>> matching_keypoints, std::vector<SRef<Point3Df>>& visiblePoints)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::GetVisible3DPoints(const SRef<Frame> frame, const std::vector<SRef<Keypoint>> keypoints, std::vector<SRef<Keypoint>> matching_keypoints, std::vector<SRef<Point3Df>>& visiblePoint, std::vector<unsigned int>& mapping)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::GetVisible3DPoint(const SRef<Frame> frame, const SRef<Keypoint> keypoint, SRef<Point3Df> visiblePoint)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::GetVisibleKeypoints(const SRef<Point3Df> point, std::vector<SRef<Frame>>& frames, std::vector<SRef<Keypoint>>& keypoints)
{
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARVisibilityStorageBiMap::GetVisibleKeypoints(const std::vector<SRef<Point3Df>>& points, std::vector<std::vector<SRef<Frame>>>& frames, std::vector<std::vector<SRef<Keypoint>>>& keypoints)
{
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
