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
namespace MODULES {
namespace TOOLS {


SolARSLAMBootstrapper::SolARSLAMBootstrapper() :ConfigurableBase(xpcf::toUUID<SolARSLAMBootstrapper>())
{
	addInterface<api::slam::IBootstrapper>(this);
	declareInjectable<api::input::devices::ICamera>(m_camera);
	declareInjectable<api::solver::pose::IFiducialMarkerPose>(m_fiducialMarkerPoseEstimator);
	declareInjectable<api::solver::map::IMapper>(m_mapper);
	declareInjectable<api::features::IKeypointDetector>(m_keypointsDetector);
	declareInjectable<api::features::IDescriptorsExtractor>(m_descriptorExtractor);
	declareInjectable<api::features::IDescriptorMatcher>(m_matcher, "Matcher");
	declareInjectable<api::features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<api::solver::map::ITriangulator>(m_triangulator);
	declareInjectable<api::solver::map::IMapFilter>(m_mapFilter);
	declareInjectable<api::solver::map::IKeyframeSelector>(m_keyframeSelector);
	declareInjectable<api::solver::map::IBundler>(m_bundler);
	declareInjectable<api::solver::pose::I3DTransformFinderFrom2D2D>(m_poseFinderFrom2D2D);
	declareInjectable<api::display::IMatchesOverlay>(m_matchesOverlay);
	declareInjectable<api::display::IImageViewer>(m_imageViewer);
	declareProperty("useMarker", m_useMarker);
	declareProperty("nbMinInitPointCloud", m_nbMinInitPointCloud);
}

void SolARSLAMBootstrapper::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_camMatrix = intrinsicParams;
	m_camDistortion = distortionParams;
	m_fiducialMarkerPoseEstimator->setCameraParameters(m_camMatrix, m_camDistortion);
	m_triangulator->setCameraParameters(m_camMatrix, m_camDistortion);
	m_poseFinderFrom2D2D->setCameraParameters(m_camMatrix, m_camDistortion);
}

FrameworkReturnCode SolARSLAMBootstrapper::run()
{
	if (m_useMarker)
		return initWithMarker();
	else
		return initWithoutMarker();
}

FrameworkReturnCode SolARSLAMBootstrapper::initWithMarker()
{
	SRef<Image>							view, imageMatches;
	Transform3Df						poseFrame;
	std::vector<Keypoint>				keypoints;
	SRef<DescriptorBuffer>				descriptors;
	std::vector<DescriptorMatch>		matches;
	SRef<Frame>							frame2;
	SRef<Keyframe>						keyframe1, keyframe2;
	std::vector<SRef<CloudPoint>>		cloud, filteredCloud;
	SRef<api::storage::IKeyframesManager>keyframesManager;
	SRef<api::reloc::IKeyframeRetriever> keyframeRetriever;
	m_mapper->getKeyframeRetriever(keyframeRetriever);
	m_mapper->getKeyframesManager(keyframesManager);

	bool bootstrapOk = false;
	bool initFrame1 = false;

	// bootstrap using marker
	while (!bootstrapOk)
	{
		if (m_camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_)
			break;

		if (m_fiducialMarkerPoseEstimator->estimate(view, poseFrame) != FrameworkReturnCode::_SUCCESS) {
			if (m_imageViewer->display(view) == SolAR::FrameworkReturnCode::_STOP)
				break;
			continue;
		}
		if (!initFrame1) {
			initFrame1 = true;
			m_keypointsDetector->detect(view, keypoints);
			m_descriptorExtractor->extract(view, keypoints, descriptors);
			keyframe1 = xpcf::utils::make_shared<Keyframe>(keypoints, descriptors, view, poseFrame);
		}
		else {
			// feature extraction
			m_keypointsDetector->detect(view, keypoints);
			m_descriptorExtractor->extract(view, keypoints, descriptors);
			frame2 = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, keyframe1, poseFrame);
			// matching
			m_matcher->match(keyframe1->getDescriptors(), frame2->getDescriptors(), matches);
			m_matchesFilter->filter(matches, matches, keyframe1->getKeypoints(), frame2->getKeypoints());
			if (matches.size() > 0) {
				m_matchesOverlay->draw(view, imageMatches, keyframe1->getKeypoints(), frame2->getKeypoints(), matches);
			}
			else {
				imageMatches = view;
				keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
			}
			if (m_imageViewer->display(imageMatches) == SolAR::FrameworkReturnCode::_STOP)
				break;
			// check baseline to triangulation
			if (m_keyframeSelector->select(frame2, matches)) {
				// Triangulate
				cloud.clear();
				filteredCloud.clear();
				m_triangulator->triangulate(keyframe1->getKeypoints(), frame2->getKeypoints(), keyframe1->getDescriptors(), frame2->getDescriptors(), matches,
					std::make_pair(0, 1), keyframe1->getPose(), frame2->getPose(), cloud);
				m_mapFilter->filter(keyframe1->getPose(), frame2->getPose(), cloud, filteredCloud);
				if (filteredCloud.size() > m_nbMinInitPointCloud) {
					// add keyframes to keyframes manager
					keyframesManager->addKeyframe(keyframe1);
					keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
					keyframesManager->addKeyframe(keyframe2);
					keyframe2->setReferenceKeyframe(keyframe1);
					// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
					for (auto const &it : filteredCloud)
						m_mapper->addCloudPoint(it);
					// add keyframes to retriever
					keyframeRetriever->addKeyframe(keyframe1);
					keyframeRetriever->addKeyframe(keyframe2);
					// apply bundle adjustement 
					double bundleReprojError = m_bundler->bundleAdjustment(m_camMatrix, m_camDistortion);
					bootstrapOk = true;
					return FrameworkReturnCode::_SUCCESS;
				}
				else {
					keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
				}
			}
		}
	}
	return FrameworkReturnCode::_ERROR_;
}

FrameworkReturnCode SolARSLAMBootstrapper::initWithoutMarker()
{
	// Initialization by get two keyframes using pose estimation from fiducial marker
	SRef<Image>							view, imageMatches;
	Transform3Df						poseFrame;
	std::vector<Keypoint>				keypoints;
	SRef<DescriptorBuffer>				descriptors;
	std::vector<DescriptorMatch>		matches;
	SRef<Frame>							frame2;
	SRef<Keyframe>						keyframe1, keyframe2;
	std::vector<SRef<CloudPoint>>		cloud, filteredCloud;
	SRef<api::storage::IKeyframesManager>keyframesManager;
	SRef<api::reloc::IKeyframeRetriever> keyframeRetriever;
	m_mapper->getKeyframeRetriever(keyframeRetriever);
	m_mapper->getKeyframesManager(keyframesManager);

	bool bootstrapOk = false;
	bool initFrame1 = false;

	// bootstrap without marker
	while (!bootstrapOk)
	{
		if (m_camera->getNextImage(view) == SolAR::FrameworkReturnCode::_ERROR_)
			break;

		if (!initFrame1) {
			initFrame1 = true;
			m_keypointsDetector->detect(view, keypoints);
			m_descriptorExtractor->extract(view, keypoints, descriptors);
			keyframe1 = xpcf::utils::make_shared<Keyframe>(keypoints, descriptors, view, Transform3Df::Identity());
		}
		else {
			// feature extraction			
			m_keypointsDetector->detect(view, keypoints);
			m_descriptorExtractor->extract(view, keypoints, descriptors);
			frame2 = xpcf::utils::make_shared<Frame>(keypoints, descriptors, view, keyframe1);
			// matching
			m_matcher->match(keyframe1->getDescriptors(), frame2->getDescriptors(), matches);
			m_matchesFilter->filter(matches, matches, keyframe1->getKeypoints(), frame2->getKeypoints());
			if (matches.size() > 0) {
				m_matchesOverlay->draw(view, imageMatches, keyframe1->getKeypoints(), frame2->getKeypoints(), matches);
			}
			else {
				imageMatches = view;				
				keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
				keyframe1->setPose(Transform3Df::Identity());
			}
			if (m_imageViewer->display(imageMatches) == SolAR::FrameworkReturnCode::_STOP)
				break;
			// check baseline to triangulation
			if (m_keyframeSelector->select(frame2, matches)) {
				// pose estimation
				m_poseFinderFrom2D2D->estimate(keyframe1->getKeypoints(), frame2->getKeypoints(), keyframe1->getPose(), poseFrame, matches);
				frame2->setPose(poseFrame);
				// Triangulate
				cloud.clear();
				filteredCloud.clear();
				m_triangulator->triangulate(keyframe1->getKeypoints(), frame2->getKeypoints(), keyframe1->getDescriptors(), frame2->getDescriptors(), matches,
					std::make_pair(0, 1), keyframe1->getPose(), frame2->getPose(), cloud);
				m_mapFilter->filter(keyframe1->getPose(), frame2->getPose(), cloud, filteredCloud);
				if (filteredCloud.size() > m_nbMinInitPointCloud) {
					// add keyframes to keyframes manager
					keyframesManager->addKeyframe(keyframe1);
					keyframe2 = xpcf::utils::make_shared<Keyframe>(frame2);
					keyframesManager->addKeyframe(keyframe2);
					keyframe2->setReferenceKeyframe(keyframe1);
					// add intial point cloud to point cloud manager and update visibility map and update covisibility graph
					for (auto const &it : filteredCloud)
						m_mapper->addCloudPoint(it);
					// add keyframes to retriever
					keyframeRetriever->addKeyframe(keyframe1);
					keyframeRetriever->addKeyframe(keyframe2);
					// apply bundle adjustement 
					double bundleReprojError = m_bundler->bundleAdjustment(m_camMatrix, m_camDistortion);
					bootstrapOk = true;
					return FrameworkReturnCode::_SUCCESS;
				}
				else {					
					keyframe1 = xpcf::utils::make_shared<Keyframe>(frame2);
					keyframe1->setPose(Transform3Df::Identity());
				}
			}
		}
	}
	return FrameworkReturnCode::_ERROR_;
}

}
}
}
