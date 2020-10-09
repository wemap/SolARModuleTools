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

#include "SolARSLAMMapping.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSLAMMapping);
#define MIN_POINT_DISTANCE 0.04


namespace SolAR {
namespace MODULES {
namespace TOOLS {


SolARSLAMMapping::SolARSLAMMapping() :ConfigurableBase(xpcf::toUUID<SolARSLAMMapping>())
{
	addInterface<api::slam::IMapping>(this);
	declareInjectable<api::solver::map::IMapper>(m_mapper);
	declareInjectable<api::storage::IPointCloudManager>(m_pointCloudManager);
	declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
	declareInjectable<api::storage::ICovisibilityGraph>(m_covisibilityGraph);
	declareInjectable<api::solver::map::IKeyframeSelector>(m_keyframeSelector);
	declareInjectable<api::solver::map::IBundler>(m_bundler);
	declareInjectable<api::reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::map::ITriangulator>(m_triangulator);
	declareInjectable<api::solver::map::IMapFilter>(m_mapFilter);
	declareInjectable<api::geom::IProject>(m_projector);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher, "Matcher-Mapping");
	declareProperty("minWeightNeighbor", m_minWeightNeighbor);
	declareProperty("minTrackedPoints", m_minTrackedPoints);
}

void SolARSLAMMapping::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_triangulator->setCameraParameters(m_camMatrix, m_camDistortion);
	m_projector->setCameraParameters(m_camMatrix, m_camDistortion);
}

FrameworkReturnCode SolARSLAMMapping::process(const SRef<Frame>& frame, SRef<Keyframe>& keyframe)
{
	// find matches between current frame and its reference keyframe
	std::vector<DescriptorMatch> matches;
	std::map<uint32_t, uint32_t> frameVisibilities = frame->getVisibility();
	uint32_t refKf_id = frame->getReferenceKeyframe()->getId();
	for (const auto &it : frameVisibilities) {
		SRef<CloudPoint> cp;
		if (m_pointCloudManager->getPoint(it.second, cp) == FrameworkReturnCode::_SUCCESS) {
			std::map<uint32_t, uint32_t> cpVisibilities = cp->getVisibility();
			auto refKf_it = cpVisibilities.find(refKf_id);
			if (refKf_it != cpVisibilities.end())
				matches.push_back(DescriptorMatch(refKf_it->second, it.first, 0.f));
		}
	}
	//LOG_INFO("Nb matches to reference keyframe: {}", matches.size());
	// check need new keyframe
	if (m_keyframeSelector->select(frame, matches) || (frame->getVisibility().size() < m_minTrackedPoints))
	{
		//LOG_INFO("Pass first condition to need new keyframe");
		if (!checkNeedNewKeyframeInLocalMap(frame)) {
			keyframe = m_updatedReferenceKeyframe;
			return FrameworkReturnCode::_ERROR_;
		}
		else {
			// create new keyframe
			keyframe = processNewKeyframe(frame);
			return FrameworkReturnCode::_SUCCESS;
		}
	}
	return FrameworkReturnCode::_ERROR_;
}

SRef<Keyframe> SolARSLAMMapping::processNewKeyframe(const SRef<Frame>& frame)
{
	// create a new keyframe from the current frame
	SRef<Keyframe> newKeyframe = xpcf::utils::make_shared<Keyframe>(frame);
	// Add to keyframe manager
	m_keyframesManager->addKeyframe(newKeyframe);
	// Add to BOW retrieval			
	m_keyframeRetriever->addKeyframe(newKeyframe);
	// Update keypoint visibility, descriptor in cloud point and connections between new keyframe with other keyframes
	updateAssociateCloudPoint(newKeyframe);
	// get best neighbor keyframes
	std::vector<uint32_t> idxBestNeighborKfs;
	m_covisibilityGraph->getNeighbors(newKeyframe->getId(), m_minWeightNeighbor, idxBestNeighborKfs);
	// find matches between unmatching keypoints in the new keyframe and the best neighboring keyframes
	std::vector<SRef<CloudPoint>> newCloudPoint;
	findMatchesAndTriangulation(newKeyframe, idxBestNeighborKfs, newCloudPoint);
	// fuse duplicate points
	//if (newCloudPoint.size() > 0)
	//	fuseCloudPoint(newKeyframe, idxBestNeighborKfs, newCloudPoint);
	LOG_INFO("Nb of new 3D points: {}", newCloudPoint.size());
	// add new points to point cloud manager, update visibility map and covisibility graph
	for (auto const &point : newCloudPoint)
		m_mapper->addCloudPoint(point);
	return newKeyframe;
}

bool SolARSLAMMapping::checkNeedNewKeyframeInLocalMap(const SRef<Frame>& frame)
{
	std::vector < uint32_t> ret_keyframesId, neighborsKfs;
	const SRef<Keyframe> &referenceKeyframe = frame->getReferenceKeyframe();
	m_covisibilityGraph->getNeighbors(referenceKeyframe->getId(), m_minWeightNeighbor, neighborsKfs);
	neighborsKfs.push_back(referenceKeyframe->getId());
	std::set<uint32_t> candidates;
	for (const auto &it : neighborsKfs)
		candidates.insert(it);
	if (m_keyframeRetriever->retrieve(frame, candidates, ret_keyframesId) == FrameworkReturnCode::_SUCCESS) {
		if (ret_keyframesId[0] != referenceKeyframe->getId()) {
			m_keyframesManager->getKeyframe(ret_keyframesId[0], m_updatedReferenceKeyframe);
			LOG_INFO("Update new reference keyframe with id {}", m_updatedReferenceKeyframe->getId());
			return false;
		}
		else {
			LOG_INFO("Find same reference keyframe with id {}", referenceKeyframe->getId());
		}		
	}
	LOG_INFO("Need to make new keyframe");
	return true;
}

void SolARSLAMMapping::updateAssociateCloudPoint(const SRef<Keyframe>& keyframe)
{
	const std::map<uint32_t, uint32_t> &newkf_mapVisibility = keyframe->getVisibility();
	std::map<uint32_t, int> kfCounter;
	// calculate the number of connections to other keyframes
	for (auto const &it : newkf_mapVisibility) {
		SRef<CloudPoint> cloudPoint;
		if (m_pointCloudManager->getPoint(it.second, cloudPoint) == FrameworkReturnCode::_SUCCESS) {
			const std::map<uint32_t, uint32_t> &cpKfVisibility = cloudPoint->getVisibility();
			for (auto const &it_kf : cpKfVisibility)
				kfCounter[it_kf.first]++;
			// add new visibility to cloud point
			cloudPoint->addVisibility(keyframe->getId(), it.first);
			// update view direction
			Transform3Df poseNewKf = keyframe->getPose();
			Vector3f newViewDirection(poseNewKf(0, 3) - cloudPoint->getX(), poseNewKf(1, 3) - cloudPoint->getY(), poseNewKf(2, 3) - cloudPoint->getZ());
			cloudPoint->addNewViewDirection(newViewDirection);
			// update descriptor
			cloudPoint->addNewDescriptor(keyframe->getDescriptors()->getDescriptor(it.first));
		}
	}

	// Add to covisibility graph
	for (auto const &it : kfCounter)
		if (it.first != keyframe->getId())
			m_covisibilityGraph->increaseEdge(keyframe->getId(), it.first, it.second);
}

void SolARSLAMMapping::findMatchesAndTriangulation(const SRef<Keyframe>& keyframe, const std::vector<uint32_t>& idxBestNeighborKfs, std::vector<SRef<CloudPoint>>& cloudPoint)
{
	const std::map<unsigned int, unsigned int> &newKf_mapVisibility = keyframe->getVisibility();
	const SRef<DescriptorBuffer> &newKf_des = keyframe->getDescriptors();
	const std::vector<Keypoint> & newKf_kp = keyframe->getKeypoints();
	const Transform3Df& newKf_pose = keyframe->getPose();

	// Vector indices keypoints have no visibility to map point
	std::vector<bool> checkMatches(newKf_kp.size(), false);
	for (int i = 0; i < newKf_kp.size(); ++i)
		if (newKf_mapVisibility.find(i) != newKf_mapVisibility.end()) {
			checkMatches[i] = true;
		}

	// Triangulate to neighboring keyframes
	for (int i = 0; i < idxBestNeighborKfs.size(); ++i) {				
		// get neighbor keyframe i
		SRef<Keyframe> tmpKf;
		m_keyframesManager->getKeyframe(idxBestNeighborKfs[i], tmpKf);
		const Transform3Df &tmpKf_pose = tmpKf->getPose();

		// check base line
		if ((tmpKf_pose.translation() - newKf_pose.translation()).norm() < 0.1)
			continue;

		// get non map point view keypoints
		std::vector<int> newKf_indexKeypoints;
		for (int j = 0; j < checkMatches.size(); ++j)
			if (!checkMatches[j])
				newKf_indexKeypoints.push_back(j);

		// Matching based on BoW
		std::vector < DescriptorMatch> tmpMatches, goodMatches;
		m_keyframeRetriever->match(newKf_indexKeypoints, newKf_des, tmpKf, tmpMatches);
		//LOG_INFO("Idx kf: {}", idxBestNeighborKfs[i]);
		//LOG_INFO("Nb of matches: {}", tmpMatches.size());
		// matches filter based epipolar lines		
		m_matchesFilter->filter(tmpMatches, tmpMatches, newKf_kp, tmpKf->getKeypoints());
		//m_matchesFilter->filter(tmpMatches, tmpMatches, newKf_kp, tmpKf->getKeypoints(), newKf_pose, tmpKf_pose, m_camMatrix);
		//LOG_INFO("Nb of filtered matches: {}", tmpMatches.size());
		// find info to triangulate				
		const std::map<unsigned int, unsigned int> & tmpMapVisibility = tmpKf->getVisibility();
		for (int j = 0; j < tmpMatches.size(); ++j) {
			unsigned int idx_newKf = tmpMatches[j].getIndexInDescriptorA();
			unsigned int idx_tmpKf = tmpMatches[j].getIndexInDescriptorB();
			if ((!checkMatches[idx_newKf]) && (tmpMapVisibility.find(idx_tmpKf) == tmpMapVisibility.end())) {
				goodMatches.push_back(tmpMatches[j]);
			}
		}
		
		// triangulation
		std::vector<SRef<CloudPoint>> tmpCloudPoint, tmpFilteredCloudPoint;
		std::vector<int> indexFiltered;
		if (goodMatches.size() > 0)
			m_triangulator->triangulate(newKf_kp, tmpKf->getKeypoints(), newKf_des, tmpKf->getDescriptors(), goodMatches,
				std::make_pair(keyframe->getId(), idxBestNeighborKfs[i]), newKf_pose, tmpKf_pose, tmpCloudPoint);

		// filter cloud points
		if (tmpCloudPoint.size() > 0)
			m_mapFilter->filter(newKf_pose, tmpKf_pose, tmpCloudPoint, tmpFilteredCloudPoint, indexFiltered);
		//LOG_INFO("Nb of cloud point: {}", tmpFilteredCloudPoint.size());
		for (int i = 0; i < indexFiltered.size(); ++i) {
			checkMatches[goodMatches[indexFiltered[i]].getIndexInDescriptorA()] = true;
			cloudPoint.push_back(tmpFilteredCloudPoint[i]);
		}
	}
}

void SolARSLAMMapping::fuseCloudPoint(const SRef<Keyframe>& keyframe, const std::vector<uint32_t>& idxNeigborKfs, std::vector<SRef<CloudPoint>>& newCloudPoint)
{
	std::vector<bool> checkMatches(newCloudPoint.size(), true);
	std::vector<SRef<DescriptorBuffer>> desNewCloudPoint;
	for (auto const &it_cp : newCloudPoint) {
		desNewCloudPoint.push_back(it_cp->getDescriptor());
	}

	for (int i = 0; i < idxNeigborKfs.size(); ++i) {
		// get a neighbor
		SRef<Keyframe> neighborKf;
		if (m_keyframesManager->getKeyframe(idxNeigborKfs[i], neighborKf) == FrameworkReturnCode::_ERROR_)
			continue;
		const std::map<uint32_t, uint32_t> &mapVisibilitiesNeighbor = neighborKf->getVisibility();

		//  projection points
		std::vector< Point2Df > projected2DPts;
		m_projector->project(newCloudPoint, projected2DPts, neighborKf->getPose());

		std::vector<DescriptorMatch> allMatches;
		m_matcher->matchInRegion(projected2DPts, desNewCloudPoint, neighborKf, allMatches, 0);

		for (int j = 0; j < allMatches.size(); ++j) {
			int idxNewCloudPoint = allMatches[j].getIndexInDescriptorA();
			int idxKpNeighbor = allMatches[j].getIndexInDescriptorB();
			if (!checkMatches[idxNewCloudPoint])
				continue;

			const std::map<uint32_t, uint32_t> &newCloudPointVisibility = newCloudPoint[idxNewCloudPoint]->getVisibility();
			// check this cloud point is created from the same neighbor keyframe
			if (newCloudPointVisibility.find(idxNeigborKfs[i]) != newCloudPointVisibility.end())
				continue;

			// check if have a cloud point in the neighbor keyframe is coincide with this cloud point.
			auto it_cp = mapVisibilitiesNeighbor.find(idxKpNeighbor);
			if (it_cp == mapVisibilitiesNeighbor.end())
				continue;

			SRef<CloudPoint> existedCloudPoint;
			if (m_pointCloudManager->getPoint(it_cp->second, existedCloudPoint) == FrameworkReturnCode::_SUCCESS) {
				const std::map<uint32_t, uint32_t> &existedCloudPointVisibility = existedCloudPoint->getVisibility();
				bool goodCheck = true;
				for (const auto &vExistedCP : existedCloudPointVisibility)
					if (newCloudPointVisibility.find(vExistedCP.first) != newCloudPointVisibility.end()) {
						goodCheck = false;
						break;
					}
				if (goodCheck && ((*existedCloudPoint - *newCloudPoint[idxNewCloudPoint]).magnitude() < MIN_POINT_DISTANCE)) {
					// add visibilities to this cloud point and keyframes
					std::vector<uint32_t> keyframeIds;
					for (auto const &vNewCP : newCloudPointVisibility) {
						existedCloudPoint->addVisibility(vNewCP.first, vNewCP.second);
						keyframeIds.push_back(vNewCP.first);
						SRef<Keyframe> tmpKeyframe;
						m_keyframesManager->getKeyframe(vNewCP.first, tmpKeyframe);
						tmpKeyframe->addVisibility(vNewCP.second, it_cp->second);
						// modify cloud point descriptor
						existedCloudPoint->addNewDescriptor(tmpKeyframe->getDescriptors()->getDescriptor(vNewCP.second));
						// modify view direction
						Transform3Df poseTmpKf = tmpKeyframe->getPose();
						Vector3f newViewDirection(poseTmpKf(0, 3) - existedCloudPoint->getX(), poseTmpKf(1, 3) - existedCloudPoint->getY(), poseTmpKf(2, 3) - existedCloudPoint->getZ());
						existedCloudPoint->addNewViewDirection(newViewDirection);
					}
					// update covisibility graph
					m_covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[0], 1);
					m_covisibilityGraph->increaseEdge(idxNeigborKfs[i], keyframeIds[1], 1);
					m_covisibilityGraph->increaseEdge(keyframeIds[0], keyframeIds[1], 1);
					// this new cloud point is existed
					checkMatches[idxNewCloudPoint] = false;
				}
			}
		}
	}

	std::vector<std::tuple<uint32_t, int, uint32_t>> tmpInfoMatches;
	std::vector<SRef<CloudPoint>> tmpNewCloudPoint;
	for (int i = 0; i < checkMatches.size(); ++i)
		if (checkMatches[i])
			tmpNewCloudPoint.push_back(newCloudPoint[i]);
	tmpNewCloudPoint.swap(newCloudPoint);
}

}
}
}
