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

#include "SolAROverlapDetector.h"
#include "core/Log.h"


namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAROverlapDetector);


namespace SolAR {
namespace MODULES {
namespace TOOLS {

SolAROverlapDetector::SolAROverlapDetector():ConfigurableBase(xpcf::toUUID<SolAROverlapDetector>())
{
    addInterface<api::loop::IOverlapDetector>(this);
	declareInjectable<IKeyframesManager>(m_keyframesManager);
	declareInjectable<ICovisibilityGraph>(m_covisibilityGraph);
	declareInjectable<reloc::IKeyframeRetriever>(m_keyframeRetriever);
	declareInjectable<solver::pose::I3DTransformSACFinderFrom3D3D>(m_estimator3D);
	declareInjectable<features::IDescriptorMatcher>(m_matcher, "Matcher-Loop");
	declareInjectable<features::IMatchesFilter>(m_matchesFilter);
	declareInjectable<solver::pose::I3D3DCorrespondencesFinder>(m_corr3D3DFinder);
	declareInjectable<geom::I3DTransform>(m_transform3D);
	declareProperty("minNbInliers", m_NbMinInliers);
}

void SolAROverlapDetector::setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distortionParams) {
	m_estimator3D->setCameraParameters(intrinsicParams, distortionParams);
}

FrameworkReturnCode SolAROverlapDetector::setGlobalMapper(const SRef<api::solver::map::IMapper>& map){
	LOG_INFO("<Set global mapper for overlapDetector:>")
	map->getKeyframesManager(m_keyframesManager);
	LOG_INFO("	-> Setting global keyframesManager");
	map->getCovisibilityGraph(m_covisibilityGraph);
	LOG_INFO("	-> Setting global keyframesRetriever");
	map->getKeyframeRetriever(m_keyframeRetriever);
	LOG_INFO("	-> Setting global pointCloudManager");
	map->getPointCloudManager(m_pointCloudManager);
	return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAROverlapDetector::detect(SRef<api::solver::map::IMapper> &globalMap, 
												const SRef<api::solver::map::IMapper> &floatingMap,
												Transform3Df &sim3Transform,
												Transform3Df&bestGlobalPose,
												Transform3Df&bestFloatinglPose) {
	

	// get floating mapper information
	SRef<IPointCloudManager> floatingPointCloudManager;
	SRef<IKeyframesManager> floatingKeyframesManager;

	std::vector<SRef<Keyframe>> allFloatingKeyframes;
	std::vector<SRef<CloudPoint>> floatingPointCloud, globalPointCloud;

	floatingMap->getKeyframesManager(floatingKeyframesManager);
	floatingMap->getPointCloudManager(floatingPointCloudManager);

	floatingKeyframesManager->getAllKeyframes(allFloatingKeyframes);
	floatingPointCloudManager->getAllPoints(floatingPointCloud);

	// get global point cloud
	m_pointCloudManager->getAllPoints(globalPointCloud);


	LOG_INFO("floating keyframes no: {}", allFloatingKeyframes.size());
	LOG_INFO("floating cloud point: {}", floatingPointCloud.size());
	LOG_INFO("floating keyframes retriever loaded correctly");
	SRef<Keyframe> bestDetectedQueryKeyframe;
	int idx_search = 0;
	for (const auto & queryKeyframe : allFloatingKeyframes) {
		uint32_t queryKeyframeId = queryKeyframe->getId();
		std::vector<uint32_t> retKeyframesIndex;
		std::vector<uint32_t> candidatesId;
		// get candidate keyframes using BoW and covisibility graph
		m_keyframeRetriever->retrieve(SRef<Frame>(queryKeyframe), retKeyframesIndex);

		for (auto &it : retKeyframesIndex) {
			std::vector<uint32_t> paths;
			m_covisibilityGraph->getShortestPath(queryKeyframeId, it, paths);
			if (paths.size() > 3)
				candidatesId.push_back(it);
		}
		std::vector<SRef<Keyframe>> candidateKeyframes;
		std::vector<Transform3Df> candidateKeyframePoses;
		for (auto &it : candidatesId) {
			SRef<Keyframe> keyframe;
			m_keyframesManager->getKeyframe(it, keyframe);
			candidateKeyframes.push_back(keyframe);
			candidateKeyframePoses.push_back(keyframe->getPose());
		}
		// find best candidate loop detection
		Transform3Df bestTransform;
		SRef<Keyframe> bestDetectedLoopKeyframe;
		std::vector<SRef<CloudPoint>> bestFirstCloudPoints, bestSecondCloudPoints;
		std::vector<int> bestInliers;
		for (const auto &it : candidateKeyframes) {
			std::vector<DescriptorMatch> matches;
			std::vector<SRef<CloudPoint>> floatingCloudPoints, globalCloudPoints;
			std::vector<uint32_t>floatingCloudPointsIndices, globalCloudPointsIndices;
			m_matcher->match(queryKeyframe->getDescriptors(), it->getDescriptors(), matches);
			m_matchesFilter->filter(matches, matches, queryKeyframe->getKeypoints(), it->getKeypoints());
			m_corr3D3DFinder->find(queryKeyframe, it, matches, floatingCloudPointsIndices, globalCloudPointsIndices);

			floatingPointCloudManager->getPoints(floatingCloudPointsIndices, floatingCloudPoints);
			m_pointCloudManager->getPoints(globalCloudPointsIndices, globalCloudPoints);

			std::vector<Point3Df> pts1, pts2;
			pts1.resize(floatingCloudPoints.size());
			pts2.resize(floatingCloudPoints.size());
			for (int i = 0; i < floatingCloudPoints.size(); ++i) {
				pts1[i] = Point3Df(floatingCloudPoints[i]->getX(), floatingCloudPoints[i]->getY(), floatingCloudPoints[i]->getZ());
				pts2[i] = Point3Df(globalCloudPoints[i]->getX(), globalCloudPoints[i]->getY(), globalCloudPoints[i]->getZ());
			}
			Transform3Df pose;
			std::vector<int> inliers;
			
			if (m_estimator3D->estimate(pts1, pts2, pose, inliers) == FrameworkReturnCode::_SUCCESS) {
				if (inliers.size() > bestInliers.size()) {
					bestTransform = pose;
					bestDetectedLoopKeyframe = it;
					bestDetectedQueryKeyframe = queryKeyframe;
					bestFirstCloudPoints.swap(floatingCloudPoints);
					bestSecondCloudPoints.swap(globalCloudPoints);
					bestInliers.swap(inliers);
				}
			}
			
		}
		if (bestInliers.size() >= m_NbMinInliers) {
			sim3Transform = bestTransform;
			bestGlobalPose = bestDetectedLoopKeyframe->getPose();
			bestFloatinglPose = bestDetectedQueryKeyframe->getPose();
			LOG_INFO("global pose -->: \n {}", bestGlobalPose.matrix());
			LOG_INFO("floating pose -->: \n {}", bestFloatinglPose.matrix());
			return FrameworkReturnCode::_SUCCESS;
		}
		else {
			LOG_INFO("bad qury fame! keep searching");
			if (idx_search >= allFloatingKeyframes.size() - 1)
				return FrameworkReturnCode::_ERROR_;
		}
		++idx_search;
	}
	return FrameworkReturnCode::_SUCCESS;

}


FrameworkReturnCode SolAROverlapDetector::detect(SRef<api::solver::map::IMapper> &globalMap,
												const SRef<api::solver::map::IMapper> &floatingMap,
												std::vector<Transform3Df> &sim3Transform,
												std::vector<std::pair<uint32_t, uint32_t>>&overlapIndices,
												std::vector<double>&scores) {
	// get floating mapper information
	SRef<IPointCloudManager> floatingPointCloudManager;
	SRef<IKeyframesManager> floatingKeyframesManager;

	std::vector<SRef<Keyframe>> allFloatingKeyframes;
	std::vector<SRef<CloudPoint>> floatingPointCloud, globalPointCloud;

	floatingMap->getKeyframesManager(floatingKeyframesManager);
	floatingMap->getPointCloudManager(floatingPointCloudManager);

	floatingPointCloudManager->getAllPoints(floatingPointCloud);
	m_pointCloudManager->getAllPoints(globalPointCloud);

	floatingKeyframesManager->getAllKeyframes(allFloatingKeyframes);

	LOG_INFO("floating keyframes no: {}", allFloatingKeyframes.size());
	LOG_INFO("floating cloud point: {}", floatingPointCloud.size());
	LOG_INFO("floating keyframes retriever loaded correctly");
	SRef<Keyframe> bestDetectedQueryKeyframe;
	int idx_search = 0;
	for (const auto & queryKeyframe : allFloatingKeyframes) {
		uint32_t queryKeyframeId = queryKeyframe->getId();
		std::vector<uint32_t> retKeyframesIndex;
		std::vector<uint32_t> candidatesId;
		// get candidate keyframes using BoW and covisibility graph
		m_keyframeRetriever->retrieve(SRef<Frame>(queryKeyframe), retKeyframesIndex);

		for (auto &it : retKeyframesIndex) {
			std::vector<uint32_t> paths;
			m_covisibilityGraph->getShortestPath(queryKeyframeId, it, paths);
			if (paths.size() > 3)
				candidatesId.push_back(it);
		}
		std::vector<SRef<Keyframe>> candidateKeyframes;
		std::vector<Transform3Df> candidateKeyframePoses;
		for (auto &it : candidatesId) {
			SRef<Keyframe> keyframe;
			m_keyframesManager->getKeyframe(it, keyframe);
			candidateKeyframes.push_back(keyframe);
			candidateKeyframePoses.push_back(keyframe->getPose());
		}
		// find best candidate loop detection
		Transform3Df bestTransform;
		SRef<Keyframe> bestDetectedLoopKeyframe;
		std::vector<SRef<CloudPoint>> bestFirstCloudPoints, bestSecondCloudPoints;
		std::vector<int> bestInliers;
		for (const auto &it : candidateKeyframes) {
			std::vector<DescriptorMatch> matches;
			std::vector<SRef<CloudPoint>> floatingCloudPoints, globalCloudPoints;
			std::vector<uint32_t>floatingCloudPointsIndices, globalCloudPointsIndices;
			m_matcher->match(queryKeyframe->getDescriptors(), it->getDescriptors(), matches);
			m_matchesFilter->filter(matches, matches, queryKeyframe->getKeypoints(), it->getKeypoints());
			m_corr3D3DFinder->find(queryKeyframe, it, matches, floatingCloudPointsIndices, globalCloudPointsIndices);

			floatingPointCloudManager->getPoints(floatingCloudPointsIndices, floatingCloudPoints);
			m_pointCloudManager->getPoints(globalCloudPointsIndices, globalCloudPoints);

			std::vector<Point3Df> pts1, pts2;
			pts1.resize(floatingCloudPoints.size());
			pts2.resize(floatingCloudPoints.size());
			for (int i = 0; i < floatingCloudPoints.size(); ++i) {
				pts1[i] = Point3Df(floatingCloudPoints[i]->getX(), floatingCloudPoints[i]->getY(), floatingCloudPoints[i]->getZ());
				pts2[i] = Point3Df(globalCloudPoints[i]->getX(), globalCloudPoints[i]->getY(), globalCloudPoints[i]->getZ());
			}
			Transform3Df pose;
			std::vector<int> inliers;

			if (m_estimator3D->estimate(pts1, pts2, pose, inliers) == FrameworkReturnCode::_SUCCESS) {
				if (inliers.size() > bestInliers.size()) {
					bestTransform = pose;
					bestDetectedLoopKeyframe = it;
					bestDetectedQueryKeyframe = queryKeyframe;
					bestFirstCloudPoints.swap(floatingCloudPoints);
					bestSecondCloudPoints.swap(globalCloudPoints);
					bestInliers.swap(inliers);
				}
			}

		}
		if (bestInliers.size() >= m_NbMinInliers) {
			sim3Transform.push_back(bestTransform);
			uint32_t idxGlobalKeyframe = bestDetectedLoopKeyframe->getId();
			uint32_t idxFloatingKeyframe = bestDetectedQueryKeyframe->getId();
			overlapIndices.push_back(std::make_pair(idxFloatingKeyframe, idxGlobalKeyframe));
			scores.push_back(double(bestInliers.size()));
			LOG_INFO("#OVERLAP detection between: floatingKeyframe {} and globalKeyframe {} with {} inliers", idxFloatingKeyframe, idxGlobalKeyframe,bestInliers.size());

		}
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
