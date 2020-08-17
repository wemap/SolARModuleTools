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
	m_matcher->match(m_frameToTrack->getDescriptors(), frame->getDescriptors(), matches);
	m_matchesFilter->filter(matches, matches, m_frameToTrack->getKeypoints(), frame->getKeypoints());
	LOG_DEBUG("Nb of matches: {}", matches.size());

	// find 2D-3D point correspondences
	std::vector<Point2Df> pt2d;
	std::vector<Point3Df> pt3d;
	std::vector<CloudPoint> foundPoints;
	std::vector<DescriptorMatch> foundMatches;
	std::vector<DescriptorMatch> remainingMatches;
	m_corr2D3DFinder->find(m_frameToTrack, frame, matches, pt3d, pt2d, foundMatches, remainingMatches);
	LOG_DEBUG("Nb of 2D-3D correspondences: {}", pt2d.size());

	// init image to display
	displayImage = frame->getView()->copy();

	// run pnp ransac
	std::vector<Point2Df> imagePoints_inliers;
	std::vector<Point3Df> worldPoints_inliers;
	if (m_pnpRansac->estimate(pt2d, pt3d, imagePoints_inliers, worldPoints_inliers, framePose, m_lastPose) == FrameworkReturnCode::_SUCCESS) {
		LOG_DEBUG(" pnp inliers size: {} / {}", worldPoints_inliers.size(), pt3d.size());
		LOG_DEBUG("Estimated pose: \n {}", framePose.matrix());
		// Set the pose of the new frame
		frame->setPose(framePose);

		// refine pose and update map visibility of frame
		// get all keypoints of the new frame
		const std::vector<Keypoint> &keypoints = frame->getKeypoints();

		//  projection points
		std::vector< Point2Df > projected2DPts;
		m_projector->project(m_localMap, projected2DPts, frame->getPose());

		// find more inlier matches
		std::vector<SRef<DescriptorBuffer>> desAllLocalMap;
		for (auto &it_cp : m_localMap) {
			desAllLocalMap.push_back(it_cp->getDescriptor());
		}
		std::vector<DescriptorMatch> allMatches;
		m_matcher->matchInRegion(projected2DPts, desAllLocalMap, frame, allMatches, 5.f);

		// find visibility of new frame
		std::vector<Point2Df> pt2d;
		std::vector<Point3Df> pt3d;
		std::map<unsigned int, unsigned int> newMapVisibility;
		for (auto &it_match : allMatches) {
			int idx_2d = it_match.getIndexInDescriptorB();
			int idx_3d = it_match.getIndexInDescriptorA();
			pt2d.push_back(Point2Df(keypoints[idx_2d].getX(), keypoints[idx_2d].getY()));
			pt3d.push_back(Point3Df(m_localMap[idx_3d]->getX(), m_localMap[idx_3d]->getY(), m_localMap[idx_3d]->getZ()));
			newMapVisibility[idx_2d] = m_localMap[idx_3d]->getId();
		}

		// pnp optimization
		Transform3Df refinedPose;
		m_pnp->estimate(pt2d, pt3d, refinedPose, frame->getPose());
		frame->setPose(refinedPose);

		// update map visibility of current frame
		frame->addVisibilities(newMapVisibility);
		LOG_DEBUG("Nb of map visibility of frame: {}", newMapVisibility.size());

		// display tracked points
		if (m_displayTrackedPoints)
			m_overlay2D->drawCircles(pt2d, displayImage);

		LOG_DEBUG("Refined pose: \n {}", frame->getPose().matrix());
		m_lastPose = frame->getPose();

		// update frame to track
		m_frameToTrack = frame;

		// tracking is good
		m_isLostTrack = false;	

	}
	else {
		LOG_DEBUG("Pose estimation has failed");
		// lost tracking
		m_isLostTrack = true;		
		// reloc
		std::vector < uint32_t> retKeyframesId;
		if (m_keyframeRetriever->retrieve(frame, retKeyframesId) == FrameworkReturnCode::_SUCCESS) {
			LOG_DEBUG("Retrieval Success");
			LOG_DEBUG("Update reference keyframe to the best retrieval keyframe id {}", retKeyframesId[0]);
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
	m_frameToTrack = xpcf::utils::make_shared<Frame>(m_referenceKeyframe);
	m_frameToTrack->setReferenceKeyframe(m_referenceKeyframe);
	m_localMap.clear();
	// get local point cloud
	m_mapper->getLocalPointCloud(m_referenceKeyframe, m_minWeightNeighbor, m_localMap);
	m_lastPose = m_referenceKeyframe->getPose();
	m_isUpdateReferenceKeyframe = false;
}

}
}
}
