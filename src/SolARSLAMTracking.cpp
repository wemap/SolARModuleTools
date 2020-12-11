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
#define BORDER_IMAGE 50

namespace SolAR {
namespace MODULES {
namespace TOOLS {


SolARSLAMTracking::SolARSLAMTracking() :ConfigurableBase(xpcf::toUUID<SolARSLAMTracking>())
{
	addInterface<api::slam::ITracking>(this);
	declareInjectable<api::solver::map::IMapper>(m_mapper);
	declareInjectable<api::storage::IKeyframesManager>(m_keyframesManager);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::pose::I2D3DCorrespondencesFinder>(m_corr2D3DFinder);
	declareInjectable<api::solver::pose::I3DTransformFinderFrom2D3D>(m_pnp);
	declareInjectable<api::solver::pose::I3DTransformSACFinderFrom2D3D>(m_pnpRansac);
	declareInjectable<api::geom::IProject>(m_projector);
	declareInjectable<api::reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<api::display::I2DOverlay>(m_overlay2DGreen, "Green");
	declareInjectable<api::display::I2DOverlay>(m_overlay2DRed, "Red");
	declareProperty("minWeightNeighbor", m_minWeightNeighbor);
	declareProperty("thresAngleViewDirection", m_thresAngleViewDirection);
	declareProperty("displayTrackedPoints", m_displayTrackedPoints);
	declareProperty("estimatedPose", m_estimatedPose);
}

xpcf::XPCFErrorCode SolARSLAMTracking::onConfigured()
{
	LOG_DEBUG(" SolARSLAMTracking onConfigured");
	m_reprojErrorThreshold = m_mapper->bindTo<xpcf::IConfigurable>()->getProperty("reprojErrorThreshold")->getFloatingValue();
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

float cosineViewDirectionAngle(const SRef<Frame>& frame, const SRef<CloudPoint>& cp) 
{
	const Transform3Df &pose = frame->getPose();
	Vector3f frameViewDir(pose(0, 3) - cp->getX(), pose(1, 3) - cp->getY(), pose(2, 3) - cp->getZ());
	const Vector3f& cpViewDir = cp->getViewDirection();
	return cpViewDir.dot(frameViewDir.normalized());
}

FrameworkReturnCode SolARSLAMTracking::process(const SRef<Frame>& frame, SRef<Image> &displayImage)
{
	// init image to display
	displayImage = frame->getView()->copy();
	std::vector<DescriptorMatch> matches;
	if (m_isLostTrack) {
		LOG_DEBUG("Pose estimation has failed");		
		// reloc
		std::vector < uint32_t> retKeyframesId;
		if (m_keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("Successful relocalization. Update reference keyframe id: {}", retKeyframesId[0]);
			SRef<Keyframe> bestRetKeyframe;
			if (m_keyframesManager->getKeyframe(retKeyframesId[0], bestRetKeyframe) == FrameworkReturnCode::_SUCCESS)
				updateReferenceKeyframe(bestRetKeyframe);
			else
				return FrameworkReturnCode::_ERROR_;
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
	for (const auto &it : matches)
		maxMatchDistance = std::max(maxMatchDistance, it.getMatchingScore());

	// find 2D-3D point correspondences
	std::vector<Point2Df> pt2d;
	std::vector<Point3Df> pt3d;
	std::vector<CloudPoint> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	std::vector < std::pair<uint32_t, SRef<CloudPoint>>> corres2D3D;
	m_corr2D3DFinder->find(m_referenceKeyframe, frame, matches, pt3d, pt2d, corres2D3D, foundMatches, remainingMatches);
	LOG_DEBUG("Nb of 2D-3D correspondences: {}", pt2d.size());		

	// run pnp ransac
	std::vector<uint32_t> inliers;
	if (!m_estimatedPose) {
		Transform3Df framePose;
		if (m_pnpRansac->estimate(pt2d, pt3d, inliers, framePose, m_lastPose) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("Inliers / Nb of correspondences: {} / {}", inliers.size(), pt3d.size());
			LOG_DEBUG("Estimated pose: \n {}", framePose.matrix());
			// Set the pose of the new frame
			frame->setPose(framePose);
		}
		else
			return FrameworkReturnCode::_ERROR_;
	}
	else {
		// define inliers based on reprojection error
		std::vector< Point2Df > projected2DPts;
		m_projector->project(pt3d, projected2DPts, frame->getPose());
		for (int i = 0; i < projected2DPts.size(); ++i) {
			float dis = (pt2d[i] - projected2DPts[i]).norm();
			if (dis < m_reprojErrorThreshold)
				inliers.push_back(i);
		}
	}
	// refine pose and update map visibility of frame
	std::map<uint32_t, uint32_t>	newMapVisibility;
	std::vector<Point2Df>			pts2dInliers, pts2dOutliers;
	std::vector<SRef<CloudPoint>>	cloudPointsOutlier;
	std::vector<Point3Df>			pts3dInliers;
	const std::vector<Keypoint>&	keypoints = frame->getKeypoints();
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
			cloudPointsOutlier.push_back(corr2D3D.second);
			pts2dOutliers.push_back(Point2Df(keypoints[corr2D3D.first].getX(), keypoints[corr2D3D.first].getY()));
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
			if ((projected2DPts[idx].getX() > BORDER_IMAGE) && (projected2DPts[idx].getX() < imgWidth - BORDER_IMAGE) && (projected2DPts[idx].getY() > BORDER_IMAGE) && (projected2DPts[idx].getY() < imgHeight - BORDER_IMAGE)) {
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
			if (checkLocalMapInOut[i]) {
				localMapUnseenCandidates[i]->updateConfidence(true);
			}
			else {
				localMapUnseenCandidates[i]->updateConfidence(false);
				cloudPointsOutlier.push_back(localMapUnseenCandidates[i]);
				pts2dOutliers.push_back(projected2DPtsCandidates[i]);
			}
		}
	}

	// check number of visibilities
	if (newMapVisibility.size() == 0)
		return FrameworkReturnCode::_ERROR_;
	// update map visibility of current frame
	frame->addVisibilities(newMapVisibility);
	LOG_DEBUG("Nb of map visibilities of current frame: {}", newMapVisibility.size());
	// pnp optimization
	Transform3Df refinedPose;
	m_pnp->estimate(pts2dInliers, pts3dInliers, refinedPose, frame->getPose());
	frame->setPose(refinedPose);	
	m_lastPose = frame->getPose();
	LOG_DEBUG("Refined pose: \n {}", frame->getPose().matrix());	
	// display tracked points
	if (m_displayTrackedPoints) {
		m_overlay2DRed->drawCircles(pts2dOutliers, displayImage);
		m_overlay2DGreen->drawCircles(pts2dInliers, displayImage);
	}		
	// tracking is good
	m_isLostTrack = false;	
	// check to need map pruning
	if (m_mapper->pointCloudPruning(cloudPointsOutlier) > 0)
		updateLocalMap();

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
