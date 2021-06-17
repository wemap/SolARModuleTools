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

#include "SolARSLAMBootstrapper.h"
#include "core/Log.h"


namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARSLAMBootstrapper);


namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {


SolARSLAMBootstrapper::SolARSLAMBootstrapper() :ConfigurableBase(xpcf::toUUID<SolARSLAMBootstrapper>())
{
	addInterface<api::slam::IBootstrapper>(this);
	declareInjectable<api::storage::IMapManager>(m_mapManager);
	declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector);
	declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher);
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::map::ITriangulator>(m_triangulator);
	declareInjectable<api::solver::map::IMapFilter>(m_mapFilter);
	declareInjectable<api::solver::map::IKeyframeSelector>(m_keyframeSelector);
	declareInjectable<api::solver::pose::I3DTransformFinderFrom2D2D>(m_poseFinderFrom2D2D);
	declareInjectable<api::geom::IUndistortPoints>(m_undistortPoints);
	declareInjectable<api::display::IMatchesOverlay>(m_matchesOverlay);
	declareProperty("hasPose", m_hasPose);
	declareProperty("nbMinInitPointCloud", m_nbMinInitPointCloud);
	declareProperty("angleThres", m_angleThres);
	LOG_DEBUG("SolARSLAMBootstrapper constructor");	
}

xpcf::XPCFErrorCode SolARSLAMBootstrapper::onConfigured()
{
	LOG_DEBUG("SolARSLAMBootstrapper onConfigured");
	m_ratioDistanceIsKeyframe = m_keyframeSelector->bindTo<xpcf::IConfigurable>()->getProperty("minMeanDistanceIsKeyframe")->getFloatingValue();
	return xpcf::XPCFErrorCode::_SUCCESS;
}

void SolARSLAMBootstrapper::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_triangulator->setCameraParameters(m_camMatrix, m_camDistortion);
	m_poseFinderFrom2D2D->setCameraParameters(m_camMatrix, m_camDistortion);
    m_undistortPoints->setCameraParameters(m_camMatrix, m_camDistortion);
}

inline float angleCamDistance(const Transform3Df & pose1, const Transform3Df & pose2) {
	return std::acos(pose1(0, 2) * pose2(0, 2) + pose1(1, 2) * pose2(1, 2) + pose1(2, 2) * pose2(2, 2));
}

FrameworkReturnCode SolARSLAMBootstrapper::process(const SRef<Image> image, SRef<Image> &view, const Transform3Df &pose)
{
	Transform3Df						poseFrame;
	std::vector<Keypoint>				keypoints, undistortedKeypoints;
	SRef<DescriptorBuffer>				descriptors;
	std::vector<DescriptorMatch>		matches;
	SRef<Frame>							frame2;	
	std::vector<SRef<CloudPoint>>		cloud, filteredCloud;	

	// TODO: benchmark this
	// TODO: should not be part of the pipeline...
	view = image->copy();
	if (m_hasPose) {
		// TODO: not efficient, change this! (neither robust)
		if (pose.isApprox(Transform3Df::Identity()))
			return FrameworkReturnCode::_ERROR_;
		else
			poseFrame = pose;
	}
	else
		poseFrame = Transform3Df::Identity();			

	// keypoint detection
	m_keypointsDetector->detect(image, keypoints);
	// undistort keypoints
	m_undistortPoints->undistort(keypoints, undistortedKeypoints);
	// feature extraction
	m_descriptorExtractor->extract(image, keypoints, descriptors);
	// If the first key frame was not created, create it
	if (!m_initKeyframe1) {
		// init first keyframe
		m_initKeyframe1 = true;		
		m_keyframe1 = xpcf::utils::make_shared<Keyframe>(keypoints, undistortedKeypoints, descriptors, image, poseFrame);
	}
	else {
		frame2 = xpcf::utils::make_shared<Frame>(keypoints, undistortedKeypoints, descriptors, image, m_keyframe1, poseFrame);		
		// Match the descriptors of the current frame with the current keyframe
		m_matcher->match(m_keyframe1->getDescriptors(), descriptors, matches);
		// Filter the matched descriptors
		// TODO: benchmark and check this (sample version does a geometric match by computing the fundamental matrix between the two frames)
		//m_matcher->matchInRegion(m_keyframe1, frame2, matches, image->getWidth() * (m_ratioDistanceIsKeyframe + 0.01));
		m_matchesFilter->filter(matches, matches, m_keyframe1->getKeypoints(), frame2->getKeypoints());
		// If there are matches subsisting, display them
		// TODO: same remark for display
		if (matches.size() > 0) {
			m_matchesOverlay->draw(image, view, m_keyframe1->getKeypoints(), frame2->getKeypoints(), matches);
		}
		// If the number of matches is not sufficient with respect to m_nbMinInitPointCloud, set the keyframe to the current frame and begin again
		if (matches.size() < m_nbMinInitPointCloud) {
			m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
		}
		// Otherwise, check if the frame is a candidate for keyframe based on implementation of IKeyframeSelector
		else if (m_keyframeSelector->select(frame2, matches)) {
			// Find pose of the second keyframe if not has pose (normally obtained thanks to the fiducial marker)
			if (!m_hasPose) {
				// TODO: check implementation
				// TODO: recomputing fundamental matrix?
				m_poseFinderFrom2D2D->estimate(m_keyframe1->getKeypoints(), frame2->getKeypoints(), m_keyframe1->getPose(), poseFrame, matches);				
				frame2->setPose(poseFrame);
			}
			// Check if the angle between the two poses is small enough => Bootstrapping should be performed by translation only
			// TODO: shouldn't this be part of the IKeyframeSelector somehow?
			if (angleCamDistance(m_keyframe1->getPose(), frame2->getPose()) > m_angleThres)
				return FrameworkReturnCode::_ERROR_;
			// TODO: shouldn't we add a criterium based on distance?

			// Triangulate
			m_triangulator->triangulate(m_keyframe1->getKeypoints(), frame2->getKeypoints(), m_keyframe1->getDescriptors(), frame2->getDescriptors(), matches,
				std::make_pair(0, 1), m_keyframe1->getPose(), frame2->getPose(), cloud);
			// Filter cloud points
			// TODO: benchmark
			m_mapFilter->filter(m_keyframe1->getPose(), frame2->getPose(), cloud, filteredCloud);
			// If the number of filtered cloud points is not sufficient with respect to m_nbMinInitPointCloud, set the keyframe to the current frame and begin again
			if (filteredCloud.size() > m_nbMinInitPointCloud) {
				// add keyframes to map manager
				m_mapManager->addKeyframe(m_keyframe1);
				m_keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
				m_mapManager->addKeyframe(m_keyframe2);
				// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
				for (auto const &it : filteredCloud)
					m_mapManager->addCloudPoint(it);
				return FrameworkReturnCode::_SUCCESS;
			}
			else {
				// TODO: conditions and tests... return error instead of doing those comparisons?
				m_keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
				if (!m_hasPose)
					m_keyframe1->setPose(Transform3Df::Identity());
			}
		}
	}
	return FrameworkReturnCode::_ERROR_;
}

}
}
}
