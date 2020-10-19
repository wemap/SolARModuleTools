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

#include "SolARSLAMTracking.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSLAMTracking);


namespace SolAR {
namespace MODULES {
namespace TOOLS {


SolARSLAMTracking::SolARSLAMTracking() :ConfigurableBase(xpcf::toUUID<SolARSLAMTracking>())
{
	addInterface<api::slam::ITracking>(this);
	declareInjectable<api::solver::map::IMapper>(m_mapper);
	declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher, "Matcher-Tracking");
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
	declareInjectable<api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
	declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
	declareInjectable<api::geom::IProject>(m_projector);
	declareInjectable<api::reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<api::display::I2DOverlay>(m_overlay2D);
	declareProperty("minWeightNeighbor", m_minWeightNeighbor);
	declareProperty("displayTrackedPoints", m_displayTrackedPoints);
}

void SolARSLAMTracking::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_pnpRansac->setCameraParameters(m_camMatrix, m_camDistortion);
	m_pnp->setCameraParameters(m_camMatrix, m_camDistortion);
	m_projector->setCameraParameters(m_camMatrix, m_camDistortion);
}

void SolARSLAMTracking::updateReferenceKeyframe(const SRef<Keyframe>& refKeyframe)
{
	std::unique_lock<std::mutex> lock(m_refKeyframeMutex);
	m_referenceKeyframe = refKeyframe;	
	m_isUpdateReferenceKeyframe = true;
}

FrameworkReturnCode SolARSLAMTracking::process(const SRef<Frame>& frame, SRef<Image> &displayImage)
{
	std::vector<DescriptorMatch> matches;
	Transform3Df framePose;

	// update local map
	if (m_isUpdateReferenceKeyframe)
		updateLocalMap();

	// set reference keyframe for new frame
	frame->setReferenceKeyframe(m_referenceKeyframe);

	// matching feature
	m_matcher->match(m_referenceKeyframe->getDescriptors(), frame->getDescriptors(), matches);
	m_matchesFilter->filter(matches, matches, m_referenceKeyframe->getKeypoints(), frame->getKeypoints());
	//LOG_INFO("Nb of matches: {}", matches.size());
	float maxMatchDistance = -FLT_MAX;
	for (const auto &it : matches) {
		float score = it.getMatchingScore();
		if (score > maxMatchDistance)
			maxMatchDistance = score;
	}
	//LOG_INFO("Max distance matches: {} / {}", maxMatchDistance);
	// find 2D-3D point correspondences
	std::vector<Point2Df> pt2d;
	std::vector<Point3Df> pt3d;
	std::vector<CloudPoint> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
	m_corr2D3DFinder->find(m_referenceKeyframe, frame, matches, pt3d, pt2d, corres2D3D, foundMatches, remainingMatches);
	LOG_DEBUG("Nb of 2D-3D correspondences: {}", pt2d.size());

	// init image to display
	displayImage = frame->getView()->copy();

	// run pnp ransac
	std::vector<uint32_t> inliers;
	if (m_pnpRansac->estimate(pt2d, pt3d, inliers, framePose, m_lastPose) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG(" pnp inliers size: {} / {}", inliers.size(), pt3d.size());
		LOG_DEBUG("Estimated pose: \n {}", framePose.matrix());
		// Set the pose of the new frame
		frame->setPose(framePose);
		// refine pose and update map visibility of frame
		std::map<uint32_t, uint32_t> newMapVisibility;
		std::vector<Point2Df> pts2dInliers;
		std::vector<Point3Df> pts3dInliers;
		const std::vector<Keypoint> &keypoints = frame->getKeypoints();
		// find visibilities from inliers and update confidence score of cloud points
		int itInliers = 0;
		inliers.push_back(-1);
		std::set<uint32_t> idxCPSeen;
		for (int itCorr = 0; itCorr < corres2D3D.size(); ++itCorr) {
			std::pair<uint32_t, SRef<CloudPoint>> corr2D3D = corres2D3D[itCorr];
			idxCPSeen.insert(corr2D3D.second->getId());
			if (itCorr == inliers[itInliers]) { // Inliers
				newMapVisibility[corr2D3D.first] = corr2D3D.second->getId();
				corr2D3D.second->updateConfidence(true);
				pts2dInliers.push_back(Point2Df(keypoints[corr2D3D.first].getX(), keypoints[corr2D3D.first].getY()));
				pts3dInliers.push_back(Point3Df(corr2D3D.second->getX(), corr2D3D.second->getY(), corr2D3D.second->getZ()));
				itInliers++;
			}
			else { // Outliers
				corr2D3D.second->updateConfidence(false);
			}
		}

		// find other visiblities from local map
		std::vector<SRef<CloudPoint>> localMapUnseen;
		for (auto &it_cp : m_localMap)
			if (idxCPSeen.find(it_cp->getId()) == idxCPSeen.end())
				localMapUnseen.push_back(it_cp);
		//  projection points
		std::vector< Point2Df > projected2DPts;
		m_projector->project(localMapUnseen, projected2DPts, frame->getPose());		
		// find more inlier matches
		std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
		for (auto &it_cp : localMapUnseen) {
			desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
		}
		std::vector<DescriptorMatch> allMatches;
		m_matcher->matchInRegion(projected2DPts, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance);
		// find visibility of new frame				
		for (auto &it_match : allMatches) {
			int idx_2d = it_match.getIndexInDescriptorB();
			int idx_3d = it_match.getIndexInDescriptorA();
			auto it2d = newMapVisibility.find(idx_2d);
			if (it2d == newMapVisibility.end()) {
				pts2dInliers.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
				pts3dInliers.push_back(Point3Df(localMapUnseen[idx_3d]->getX(), localMapUnseen[idx_3d]->getY(), localMapUnseen[idx_3d]->getZ()));
				newMapVisibility[idx_2d] = localMapUnseen[idx_3d]->getId();
			}			
		}

		// pnp optimization
		Transform3Df refinedPose;
		m_pnp->estimate(pts2dInliers, pts3dInliers, refinedPose, frame->getPose());
		frame->setPose(refinedPose);

		// update map visibility of current frame
		frame->addVisibilities(newMapVisibility);
		LOG_DEBUG("Nb of map visibilities of frame: {}", newMapVisibility.size());

		// display tracked points
		if (m_displayTrackedPoints)
			m_overlay2D->drawCircles(pts2dInliers, displayImage);

		LOG_DEBUG("Refined pose: \n {}", frame->getPose().matrix());
		m_lastPose = frame->getPose();

		// tracking is good
		m_isLostTrack = false;	

	}
	else {
		LOG_INFO("Pose estimation has failed");
		// lost tracking
		m_isLostTrack = true;		
		// reloc
		std::vector < uint32_t> retKeyframesId;
		if (m_keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("Retrieval Success");
			LOG_INFO("Update reference keyframe to the best retrieval keyframe id {}", retKeyframesId[0]);
			SRef<Keyframe> bestRetKeyframe;
			m_keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe);
			updateReferenceKeyframe(bestRetKeyframe);		
		}
		else
			LOG_DEBUG("Retrieval Failed");
	}
	if (m_isLostTrack)
		return FrameworkReturnCode::_ERROR_;
	else
		return FrameworkReturnCode::_SUCCESS;
}

void SolARSLAMTracking::updateLocalMap()
{
	std::unique_lock<std::mutex> lock(m_refKeyframeMutex);
	m_localMap.clear();
	// get local point cloud
	m_mapper->getLocalPointCloud(m_referenceKeyframe, m_minWeightNeighbor, m_localMap);
	m_lastPose = m_referenceKeyframe->getPose();
	m_isUpdateReferenceKeyframe = false;
}

}
}
}
