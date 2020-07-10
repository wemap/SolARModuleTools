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

#include "xpcf/xpcf.h"
#include "api/solver/map/IMapper.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/solver/pose/I3DTransformSACFinderFrom3D3D.h"
#include "api/geom/I3DTransform.h"
#include "api/display/I3DPointsViewer.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I3D3DCorrespondencesFinder.h"
#include "api/features/IMatchesFilter.h"
#include "core/Log.h"
#include <boost/log/core.hpp>

using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

namespace xpcf  = org::bcom::xpcf;

int main(int argc,char** argv)
{
#if NDEBUG
    boost::log::core::get()->set_logging_enabled(false);
#endif

    LOG_ADD_LOG_TO_CONSOLE();
	/* instantiate component manager*/
	/* this is needed in dynamic mode */
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

	if (xpcfComponentManager->load("SolARTestLoopClosureDetection_config.xml") != org::bcom::xpcf::_SUCCESS)
	{
		LOG_ERROR("Failed to load the configuration file SolARTestLoopClosureDetection_config.xml")
			return -1;
	}

	// declare and create components
	LOG_INFO("Start creating components");

	// storage components
	auto pointCloudManager = xpcfComponentManager->resolve<IPointCloudManager>();
	auto keyframesManager = xpcfComponentManager->resolve<IKeyframesManager>();
	auto covisibilityGraph = xpcfComponentManager->resolve<ICovisibilityGraph>();
	auto keyframeRetriever = xpcfComponentManager->resolve<IKeyframeRetriever>();
	auto mapper = xpcfComponentManager->resolve<solver::map::IMapper>();
	auto estimator3D = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom3D3D>();
	auto transform3D = xpcfComponentManager->resolve<geom::I3DTransform>();
	auto viewer3DPoints = xpcfComponentManager->resolve<display::I3DPointsViewer>();
	auto matcher = xpcfComponentManager->resolve<features::IDescriptorMatcher>();
	auto matchesFilter = xpcfComponentManager->resolve<features::IMatchesFilter>();
	auto corr3D3DFinder = xpcfComponentManager->resolve<solver::pose::I3D3DCorrespondencesFinder>();


	// Load map from file
	if (mapper->loadFromFile() == FrameworkReturnCode::_SUCCESS) {
		LOG_INFO("Load map done!");
	}
	else {
		LOG_INFO("Cannot load map");
		return 0;
	}

	LOG_INFO("Number of keyframes: {}", keyframesManager->getNbKeyframes());
	LOG_INFO("Number of cloud points: {}", pointCloudManager->getNbPoints());

	// get all keyframes and point cloud to display
	std::vector<Transform3Df> keyframePoses;
	std::vector<SRef<Keyframe>> allKeyframes;
	keyframesManager->getAllKeyframes(allKeyframes);	
	for (auto const &it : allKeyframes)
		keyframePoses.push_back(it->getPose());
	std::vector<SRef<CloudPoint>> pointCloud;
	pointCloudManager->getAllPoints(pointCloud);

	// test loop closure with the last keyframe
	SRef<Keyframe> lastKeyframe = allKeyframes[allKeyframes.size() - 1];
	uint32_t lastKeyframeId = lastKeyframe->getId();
	std::cout << "Last keyframe id: " << lastKeyframeId << std::endl;
	std::vector<uint32_t> retKeyframesIndex;	
	std::vector<uint32_t> candidatesId;	
	// get candidate keyframes using BoW and covisibility graph
	keyframeRetriever->retrieve(SRef<Frame>(lastKeyframe), retKeyframesIndex);
	std::cout << "Retrieve keyframe id: ";
	for (auto &it : retKeyframesIndex) {
		std::cout << it << " ";
		std::vector<uint32_t> paths;
		covisibilityGraph->getShortestPath(lastKeyframeId, it, paths);
		if (paths.size() > 3)
			candidatesId.push_back(it);
	}
	std::cout << std::endl;
	std::cout << "Candidates keyframe id: ";
	std::vector<SRef<Keyframe>> candidateKeyframes;
	std::vector<Transform3Df> candidateKeyframePoses;
	for (auto &it : candidatesId) {
		std::cout << it << " ";
		SRef<Keyframe> keyframe;
		keyframesManager->getKeyframe(it, keyframe);
		candidateKeyframes.push_back(keyframe);
		candidateKeyframePoses.push_back(keyframe->getPose());
	}
	std::cout << std::endl;

	// find best candidate loop detection
	Transform3Df bestTransform;
	SRef<Keyframe> bestDetectedLoopKeyframe;
	std::vector<SRef<CloudPoint>> bestFirstCloudPoints, bestSecondCloudPoints;
	std::vector<int> bestInliers;
	for (const auto &it : candidateKeyframes) {
		std::cout << std::endl << "Process candidate: " << it->getId() << std::endl;
		std::vector<DescriptorMatch> matches, foundMatches, remainingMatches;		
		std::vector<SRef<CloudPoint>> firstCloudPoints, secondCloudPoints;
		matcher->match(lastKeyframe->getDescriptors(), it->getDescriptors(), matches);
		std::cout << "Number of matches: " << matches.size() << std::endl;
		matchesFilter->filter(matches, matches, lastKeyframe->getKeypoints(), it->getKeypoints());
		std::cout << "Number of matches filter: " << matches.size() << std::endl;
		corr3D3DFinder->find(lastKeyframe, it, matches, firstCloudPoints, secondCloudPoints, foundMatches, remainingMatches);
		std::cout << "Number of 3D-3D finder: " << firstCloudPoints.size() << std::endl;	
		std::vector<Point3Df> pts1, pts2;
		pts1.resize(firstCloudPoints.size());
		pts2.resize(firstCloudPoints.size());
		for (int i = 0; i < firstCloudPoints.size(); ++i) {
			pts1[i] = Point3Df(firstCloudPoints[i]->getX(), firstCloudPoints[i]->getY(), firstCloudPoints[i]->getZ());
			pts2[i] = Point3Df(secondCloudPoints[i]->getX(), secondCloudPoints[i]->getY(), secondCloudPoints[i]->getZ());
		}
		Transform3Df pose;
		std::vector<int> inliers;
		if (estimator3D->estimate(pts1, pts2, pose, inliers) == FrameworkReturnCode::_ERROR_) {
			LOG_ERROR("Cannot find 3D transformation");
		}
		else {
			std::cout << "Inliers: " << inliers.size() << std::endl;
			if (inliers.size() > bestInliers.size()) {
				bestTransform = pose;
				bestDetectedLoopKeyframe = it;
				bestFirstCloudPoints.swap(firstCloudPoints);
				bestSecondCloudPoints.swap(secondCloudPoints);
				bestInliers.swap(inliers);
			}
		}
	}

	// try to transform local point cloud of current keyframe to the detected loop keyframe
	std::vector<SRef<CloudPoint>> localPointCloud, localPointCloudTrans;
	std::vector<Point3Df> localPoint3D, localPoint3DTrans;
	mapper->getLocalPointCloud(lastKeyframe, 10, localPointCloud);
	for (auto it : localPointCloud)
		localPoint3D.push_back(Point3Df(it->getX(), it->getY(), it->getZ()));
	transform3D->transform(localPoint3D, bestTransform, localPoint3DTrans);
	for (auto it : localPoint3DTrans)
		localPointCloudTrans.push_back(xpcf::utils::make_shared<CloudPoint>(it.getX(), it.getY(), it.getZ()));

	// best detected loop keyframe
	std::cout << "===================== Best detected loop keyframe ======================" << std::endl;
	std::cout << "Keyframe id: " << bestDetectedLoopKeyframe->getId() << std::endl;
	std::cout << "Transform 3D from last keyframe and best detected loop keyframe: " << std::endl;
	std::cout << bestTransform.matrix() << std::endl;
	std::cout << bestTransform.rotation() << std::endl;
	std::cout << bestTransform.translation() << std::endl;

	// display point cloud
	while (viewer3DPoints->display(localPointCloudTrans, lastKeyframe->getPose(), {bestDetectedLoopKeyframe->getPose()}, {}, pointCloud, keyframePoses) == FrameworkReturnCode::_SUCCESS);

    return 0;
}





