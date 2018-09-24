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

#include "SolARMapper.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapper)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARMapper::SolARMapper():ComponentBase(xpcf::toUUID<SolARMapper>())
    {
        addInterface<IMapper>(this);
        m_map = xpcf::utils::make_shared<Map>() ;
    }

    SRef<Map> SolARMapper::getMap()
    {
        return m_map ;
    }

    FrameworkReturnCode SolARMapper::update (SRef<Map> map,
                                             SRef<Keyframe> newKeyframe,
                                             const std::vector<SRef<CloudPoint>>& newCloud,
                                             const std::vector<DescriptorMatch>& newPointsMatches,
                                             const std::vector<DescriptorMatch>& existingPointsMatches)
    {
        if (m_kframes.size() == 0)
        {
            if (newCloud.size() != 0 || newPointsMatches.size() != 0 || existingPointsMatches.size() != 0)
            {
                LOG_WARNING("For the first update of the Mapper, only the first keyframe is required");
            }
            m_kframes.push_back(newKeyframe);
            map = m_map;
            return FrameworkReturnCode::_SUCCESS;
        }
        if (m_kframes.size() == 1)
        {
            if (existingPointsMatches.size() != 0)
                LOG_WARNING("For the second update of the Mapper, not need of existing");

            newKeyframe->getReferenceKeyframe()->addVisibleMapPoints(newCloud);
            newKeyframe->addVisibleMapPoints(newCloud);
            m_gmatches[std::make_pair(newKeyframe->getReferenceKeyframe()->m_idx, newKeyframe->m_idx)] = newPointsMatches;
            m_map->addCloudPoints(newCloud) ;
            return FrameworkReturnCode::_SUCCESS;
        }

        // A point cloud exist and we need to check if matches have already a corresponding 3D points
        std::vector<SRef<CloudPoint>> previous_cloud;
        SRef<std::vector<SRef<CloudPoint>>> pointCloud = m_map->getPointCloud();
        m_gmatches[std::make_pair(newKeyframe->getReferenceKeyframe()->m_idx, newKeyframe->m_idx)] = newPointsMatches;
        for (int i = 0; i < existingPointsMatches.size(); i++)
        {
            for (int j = 0; j < pointCloud->size(); j++)
            {
                if ((*pointCloud)[j]->m_visibility[newKeyframe->getReferenceKeyframe()->m_idx] == existingPointsMatches[i].getIndexInDescriptorA())
                {
                    // Update the visibility of the existing 3D point with the new keyframe
                    (*pointCloud)[j]->m_visibility[newKeyframe->m_idx] = existingPointsMatches[i].getIndexInDescriptorB();
                    // Add the 3D point to the point cloud which will be attach to the new keyframe
                    previous_cloud.push_back((*pointCloud)[j]);
                    break;
                }
            }
        }
        newKeyframe->addVisibleMapPoints(previous_cloud);

        // Add the 3D points that have just been triangulated
        m_map->addCloudPoints(newCloud);
        return FrameworkReturnCode::_SUCCESS;
    }
}
}
}
