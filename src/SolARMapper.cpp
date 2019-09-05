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
        std::map<unsigned int, unsigned int>::const_iterator visIt;

        for (int i = 0; i < newCloud.size(); i++)
        {
            const std::map<unsigned int, unsigned int> & cloudPointVisibility = newCloud[i].getVisibility();
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
            for ( auto & currentExistingMatch : existingPointsMatches) {
                // update the existing 3D points already in the map visible by the current keyframe and reciprocally
                std::map<unsigned int, unsigned int>::iterator refKFVisIt = refKeyframeVisibility.find(currentExistingMatch.getIndexInDescriptorA());
                if ( refKFVisIt != refKeyframeVisibility.end() ) {
                    keyframeVisibility[currentExistingMatch.getIndexInDescriptorB()] = refKFVisIt->second;
                    m_map->getAPoint(refKFVisIt->second).visibilityAddKeypoint(newKeyframe->m_idx, currentExistingMatch.getIndexInDescriptorB());
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

    FrameworkReturnCode SolARMapper::update(const std::vector<CloudPoint> & correctedCloud,
                                            const std::vector<SRef<Keyframe>> & correctedKeyframes) {
        //if (m_kframes.size() =! correctedKeyframes.size())
        //{
        //	if (existingPointsMatches.size() != 0)
        //	{
        //		LOG_WARNING("For the second update of the Mapper, not need of existing points");
        //	}
        //}
        // update keyframes:
        //	# update poses and leave other members fixed (descriptors, keypoints..etc).
        for (unsigned int j = 0; j < m_kframes.size(); ++j) {
            m_kframes[j]->setPose(correctedKeyframes[j]->getPose());
        }
        // update map
        //	# update cloud point and leave other members fixed (visibility..etc)
        m_map->updateCloudPoints(correctedCloud);
        return FrameworkReturnCode::_SUCCESS;
    }

    void SolARMapper::getLocalMap(SRef<Keyframe> refKF, std::vector<CloudPoint> &localCloudPoints)
    {
        // Get all 3D cloud points seen from refKF and its neighbors

        std::set<unsigned int> idxPC;
        localCloudPoints.clear();
        const std::map<unsigned int, unsigned int> & cpRefKF = refKF->getVisibleMapPoints();
        for (auto & kv : cpRefKF) {
            idxPC.insert(kv.second);
        }

        const std::map<unsigned int, unsigned int> & neighbors = refKF->getNeighborKeyframes();

        for (auto & kv  : neighbors) {
            std::map<unsigned int, unsigned int> neighborVisibility = m_kframes[kv.first]->getVisibleMapPoints();
            for (auto & kvNeighborVis : neighborVisibility) {
                idxPC.insert(kvNeighborVis.second);
            }
        }

        std::vector<CloudPoint> pointCloud = m_map->getPointCloud();

        for (auto index : idxPC) {
            localCloudPoints.push_back(pointCloud[index]);
        }
    }

    SRef<Map> SolARMapper::getGlobalMap() {
        return m_map;
    }

    void SolARMapper::getLocalMapIndex(SRef<Keyframe> refKF, std::vector<unsigned int>& idxLocalCloudPoints)
    {
        // Get all index of 3D cloud points seen from refKF and its neighbors

        idxLocalCloudPoints.clear();

        std::set<unsigned int> idxPC;

        const std::map<unsigned int, unsigned int> & cpRefKF = refKF->getVisibleMapPoints();
        for (auto & kv : cpRefKF) {
            idxPC.insert(kv.second);
        }

        const std::map<unsigned int, unsigned int> & neighbors = refKF->getNeighborKeyframes();

        for (auto & kv : neighbors) {
            const std::map<unsigned int, unsigned int> & neighborVisibility = m_kframes[kv.first]->getVisibleMapPoints();
            for (auto & kvVisMap : neighborVisibility) {
                idxPC.insert(kvVisMap.second);
            }
        }

        idxLocalCloudPoints.assign(idxPC.begin(), idxPC.end());
    }

    }
    }
}
