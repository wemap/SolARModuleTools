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
#include "core/Log.h"

namespace xpcf  = org::bcom::xpcf;


XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARMapper)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARMapper::SolARMapper():ComponentBase(xpcf::toUUID<SolARMapper>())
    {
        declareInterface<IMapper>(this);
        m_map = xpcf::utils::make_shared<Map>();
    }

    FrameworkReturnCode SolARMapper::update (SRef<Map> & map,
                                             SRef<Keyframe> & newKeyframe,
                                             const std::vector<CloudPoint> & newCloud,
                                             const std::vector<DescriptorMatch> & newPointsMatches,
                                             const std::vector<DescriptorMatch> & existingPointsMatches)
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

        // A point cloud exist and we need to check if matches have already a corresponding 3D points
        SRef<Keyframe> referenceKeyframe = newKeyframe->getReferenceKeyframe();
        if (referenceKeyframe == nullptr)
        {
            LOG_ERROR("When updating the map, if the keyframe is not the first one, it should have a reference keyframe");
            map = m_map;
            return FrameworkReturnCode::_ERROR_;
        }

		int idxLastCP = m_map->getPointCloud().size();

        std::map<unsigned int, unsigned int> keyframeVisibility, newRefKeyframeVisibility, refKeyframeVisibility = referenceKeyframe->getVisibleMapPoints();
        std::map<unsigned int, unsigned int>::iterator visIt;

        for (int i = 0; i < newCloud.size(); i++)
        {
            std::map<unsigned int, unsigned int> cloudPointVisibility = newCloud[i].getVisibility();
            // update the visibility of the current keyframe with the new 3D points
            visIt = cloudPointVisibility.find(newKeyframe->m_idx);
            if (visIt != cloudPointVisibility.end())
            {
                keyframeVisibility[visIt->second] = idxLastCP + i;
            }
            // update the visibility of the reference keyframe with the new 3D points
            visIt = cloudPointVisibility.find(referenceKeyframe->m_idx);
            if (visIt != cloudPointVisibility.end())
            {
                newRefKeyframeVisibility[visIt->second] = idxLastCP + i;
            }
        }

        if (m_kframes.size() == 1)
        {
            if (existingPointsMatches.size() != 0)
            {
                LOG_WARNING("For the second update of the Mapper, not need of existing points");
            }
        }
        else
        {
            for (int i = 0; i < existingPointsMatches.size(); i++)
            {
                // update the existing 3D points already in the map visible by the current keyframe and reciprocally
                std::map<unsigned int, unsigned int>::iterator refKFVisIt = refKeyframeVisibility.find(existingPointsMatches[i].getIndexInDescriptorA());
                if ( refKFVisIt != refKeyframeVisibility.end() )
                {
                    keyframeVisibility[existingPointsMatches[i].getIndexInDescriptorB()] = refKFVisIt->second;
					m_map->getAPoint(refKFVisIt->second).visibilityAddKeypoint(newKeyframe->m_idx, existingPointsMatches[i].getIndexInDescriptorB());
                }
            }
        }
        m_gmatches[std::make_pair(referenceKeyframe->m_idx, newKeyframe->m_idx)] = newPointsMatches;
        newKeyframe->addVisibleMapPoints(keyframeVisibility);
        referenceKeyframe->addVisibleMapPoints(newRefKeyframeVisibility);
        m_kframes.push_back(newKeyframe);
        // Add the 3D points that have just been triangulated
        m_map->addCloudPoints(newCloud);
        map = m_map;
        return FrameworkReturnCode::_SUCCESS;
    }

	void SolARMapper::getLocalMap(SRef<Keyframe> refKF, std::vector<CloudPoint> &localCloudPoints)
	{
		// the initial localCloudPoints consists of 3D points seen from refKF
		// if the initial localCloudPoints is empty, get all 3D cloud points seen from refKF and its neighbors 
		// else add only 3D points of refKF's neighbors to localCloudPoints 				

		std::set<unsigned int> idxPC;

		if (localCloudPoints.empty()) {
			std::map<unsigned int, unsigned int> cpRefKF = refKF->getVisibleMapPoints();			
			for (auto it = cpRefKF.begin(); it != cpRefKF.end(); it++)
				idxPC.insert(it->second);
		}

		std::map<unsigned int, unsigned int> neighbors = refKF->getNeighborKeyframes();

		for (auto it_nb = neighbors.begin(); it_nb != neighbors.end(); it_nb++) {
			std::map<unsigned int, unsigned int> neighborVisibility = m_kframes[it_nb->first]->getVisibleMapPoints();
			for (auto it_vi = neighborVisibility.begin(); it_vi != neighborVisibility.end(); it_vi++) {
				idxPC.insert(it_vi->second);
			}
		}

		std::vector<CloudPoint> pointCloud = m_map->getPointCloud();

		for (auto it = idxPC.begin(); it != idxPC.end(); it++)
			localCloudPoints.push_back(pointCloud[*it]);
	}

}
}
}
