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
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARSLAMTracking::SolARSLAMTracking() :ConfigurableBase(xpcf::toUUID<SolARSLAMTracking>())
{
    addInterface<SolAR::api::slam::ITracking>(this);
    declareInjectable<SolAR::api::solver::map::IMapper>(m_mapper);
    declareInjectable<SolAR::api::storage::IKeyframesManager>(m_keyframesManager);
    declareInjectable<SolAR::api::features::IDescriptorMatcher>(m_matcher);
    declareInjectable<SolAR::api::features::IMatchesFilter>(m_matchesFilter);
    declareInjectable<SolAR::api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
    declareInjectable<SolAR::api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
    declareInjectable<SolAR::api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
    declareInjectable<SolAR::api::geom::IProject>(m_projector);
    declareInjectable<SolAR::api::reloc::IKeyframeRetriever>(m_keyframeRetriever);
    declareInjectable<SolAR::api::display::I2DOverlay>(m_overlay2D);
	declareProperty("minWeightNeighbor", m_minWeightNeighbor);
	declareProperty("thresAngleViewDirection", m_thresAngleViewDirection);
	declareProperty("displayTrackedPoints", m_displayTrackedPoints);
}

xpcf::XPCFErrorCode SolARSLAMTracking::onConfigured()
{
	LOG_DEBUG("SolARSLAMTracking onConfigured");
	m_reprojErrorThreshold = m_mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
	return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARSLAMTracking::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_pnpRansac->setCameraParameters(m_camMatrix, m_camDistortion);
	m_pnp->setCameraParameters(m_camMatrix, m_camDistortion);
	m_projector->setCameraParameters(m_camMatrix, m_camDistortion);	
}

void SolARSLAMTracking::updateReferenceKeyframe(const SRef<Keyframe> refKeyframe)
{
	std::unique_lock<std::mutex> lock(m_refKeyframeMutex);
	m_referenceKeyframe = refKeyframe;	
	m_isUpdateReferenceKeyframe = true;
}

float cosineViewDirectionAngle(const SRef<Frame>& frame, const SRef<CloudPoint>& cp) 
{
	const Transform3Df &pose = frame->getPose();
	Vector3f frameViewDir(pose(0, 3) - cp->getX(), pose(1, 3) - cp->getY(), pose(2, 3) - cp->getZ());
	const Vector3f& cpViewDir = cp->getViewDirection();
	return cpViewDir.dot(frameViewDir.normalized());
};

FrameworkReturnCode SolARSLAMTracking::process(const SRef<Frame> frame, SRef<Image> &displayImage)
{
	// init image to display
	displayImage = frame->getView()->copy();
	std::vector<DescriptorMatch> matches;
	Transform3Df framePose = frame->getPose();
	if (m_isLostTrack) {
		LOG_DEBUG("Pose estimation has failed");		
		// reloc
		std::vector < uint32_t> retKeyframesId;
		if (m_keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("Successful relocalization. Update reference keyframe id: {}", retKeyframesId[0]);
			SRef<Keyframe> bestRetKeyframe;
			m_keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe);
			updateReferenceKeyframe(bestRetKeyframe);
		}
		else
			LOG_DEBUG("Relocalization Failed");
	}
	// update local map
	if (m_isUpdateReferenceKeyframe) {
		updateLocalMap();
	}

	// set reference keyframe for new frame
	frame->setReferenceKeyframe(m_referenceKeyframe);	
	// matching feature
	m_isLostTrack = true;
	m_matcher->match(m_referenceKeyframe->getDescriptors(), frame->getDescriptors(), matches);
	if (matches.size() < 10)
		return FrameworkReturnCode::_ERROR_;
	m_matchesFilter->filter(matches, matches, m_referenceKeyframe->getKeypoints(), frame->getKeypoints());
	float maxMatchDistance = -FLT_MAX;
	for (const auto &it : matches) {
		float score = it.getMatchingScore();
		if (score > maxMatchDistance)
			maxMatchDistance = score;
	}
	// find 2D-3D point correspondences
	std::vector<Point2Df> pt2d;
	std::vector<Point3Df> pt3d;
	std::vector<CloudPoint> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
	m_corr2D3DFinder->find(m_referenceKeyframe, frame, matches, pt3d, pt2d, corres2D3D, foundMatches, remainingMatches);
	LOG_DEBUG("Nb of 2D-3D correspondences: {}", pt2d.size());	
	if (pt2d.size() == 0)
		return FrameworkReturnCode::_ERROR_;

	// find initial pose
	bool bFindPose = framePose.isApprox(Transform3Df::Identity());
	if (bFindPose) {
		std::vector<uint32_t> inliers;
		if (m_pnpRansac->estimate(pt2d, pt3d, inliers, framePose, m_lastPose) != FrameworkReturnCode::_SUCCESS)
			return FrameworkReturnCode::_ERROR_;
		LOG_DEBUG("Inliers / Nb of correspondences: {} / {}", inliers.size(), pt3d.size());
		LOG_DEBUG("Estimated pose: \n {}", framePose.matrix());		
		frame->setPose(framePose);	// Set the pose of the new frame
	}

	// Update map visibility of the current frame
	// And find more inliers of 2D-3D points for improving pose
	std::map<uint32_t, uint32_t> newMapVisibility;
	std::vector<Point2Df> pts2dInliers;
	std::vector<Point3Df> pts3dInliers;
	const std::vector<Keypoint> &keypoints = frame->getKeypoints();
	// Define inlier/outlier of 2D-3D correspondences found by the reference keyframe
	std::vector< Point2Df > pt3DProj;
	std::set<uint32_t> idxCPSeen;
	m_projector->project(pt3d, pt3DProj, framePose);
	for (int i = 0; i < pt3DProj.size(); ++i) {
		idxCPSeen.insert(corres2D3D[i].second->getId());
		float dis = (pt2d[i] - pt3DProj[i]).norm();
		if (dis < m_reprojErrorThreshold) {
			corres2D3D[i].second->updateConfidence(true);
			newMapVisibility[corres2D3D[i].first] = corres2D3D[i].second->getId();
			pts2dInliers.push_back(pt2d[i]);
			pts3dInliers.push_back(pt3d[i]);
		}
		else {
			corres2D3D[i].second->updateConfidence(false);
		}
	}

	// find other visiblities from local map
	std::vector<SRef<CloudPoint>> localMapUnseen;
	for (auto &it_cp : m_localMap)
		if ((idxCPSeen.find(it_cp->getId()) == idxCPSeen.end()) && (cosineViewDirectionAngle(frame, it_cp) > m_thresAngleViewDirection))
			localMapUnseen.push_back(it_cp);

	std::vector<SRef<CloudPoint>> localMapUnseenCandidates;
	std::vector< Point2Df > projected2DPtsCandidates;
	//  projection points and filter point out of frame
	if (localMapUnseen.size() > 0) {
		std::vector< Point2Df > projected2DPts;
		m_projector->project(localMapUnseen, projected2DPts, frame->getPose());
		uint32_t imgWidth = frame->getView()->getWidth();
		uint32_t imgHeight = frame->getView()->getHeight();
		for (int idx = 0; idx < projected2DPts.size(); idx++)
			if ((projected2DPts[idx].getX() > 0) && (projected2DPts[idx].getX() < imgWidth) && (projected2DPts[idx].getY() > 0) && (projected2DPts[idx].getY() < imgHeight)) {
				projected2DPtsCandidates.push_back(std::move(projected2DPts[idx]));
				localMapUnseenCandidates.push_back(std::move(localMapUnseen[idx]));
			}
	}
	LOG_DEBUG("Nb of filtered local map : {}", localMapUnseenCandidates.size());

	if (localMapUnseenCandidates.size() > 0) {
		// find more inlier matches
		std::vector<SRef<DescriptorBuffer>> desAllLocalMapUnseen;
		for (auto &it_cp : localMapUnseenCandidates) {
			desAllLocalMapUnseen.push_back(it_cp->getDescriptor());
		}
		std::vector<DescriptorMatch> allMatches;
		m_matcher->matchInRegion(projected2DPtsCandidates, desAllLocalMapUnseen, frame, allMatches, 0, maxMatchDistance);
		// find visibility of new frame
		int nbMatchesLocalMap(0);
		std::vector<bool> checkLocalMapInOut(localMapUnseenCandidates.size(), false);
		for (auto &it_match : allMatches) {
			int idx_2d = it_match.getIndexInDescriptorB();
			int idx_3d = it_match.getIndexInDescriptorA();
			checkLocalMapInOut[idx_3d] = true;
			if (newMapVisibility.find(idx_2d) == newMapVisibility.end()) {
				pts2dInliers.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
				pts3dInliers.push_back(Point3Df(localMapUnseenCandidates[idx_3d]->getX(), localMapUnseenCandidates[idx_3d]->getY(), localMapUnseenCandidates[idx_3d]->getZ()));
				newMapVisibility[idx_2d] = localMapUnseenCandidates[idx_3d]->getId();
				nbMatchesLocalMap++;
			}
		}
		LOG_DEBUG("Nb of matched local map: {}", nbMatchesLocalMap);
		// update confidence score of matched cloud points
		for (int i = 0; i < localMapUnseenCandidates.size(); ++i) {
			if (checkLocalMapInOut[i])
				localMapUnseenCandidates[i]->updateConfidence(true);
		}
	}

	// update map visibility of current frame
	frame->addVisibilities(newMapVisibility);
	LOG_DEBUG("Nb of map visibilities of current frame: {}", newMapVisibility.size());

	// pnp optimization
	if (bFindPose) {
		Transform3Df refinedPose;
		m_pnp->estimate(pts2dInliers, pts3dInliers, refinedPose, frame->getPose());
		frame->setPose(refinedPose);
	}

	// display tracked points
	if (m_displayTrackedPoints)
		m_overlay2D->drawCircles(pts2dInliers, displayImage);

	LOG_DEBUG("Refined pose: \n {}", frame->getPose().matrix());
	m_lastPose = frame->getPose();

	// tracking is good
	m_isLostTrack = false;
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
