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
#include "api/solver/pose/I3DTransformSACFinderFrom3D3D.h"
#include "api/geom/I3DTransform.h"
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

	if (xpcfComponentManager->load("SolARTest3DTransformEstimationSACFrom3D3D_config.xml") != org::bcom::xpcf::_SUCCESS)
	{
		LOG_ERROR("Failed to load the configuration file SolARTest3DTransformEstimationSACFrom3D3D_config.xml")
			return -1;
	}

	// declare and create components
	LOG_INFO("Start creating components");
	auto estimator3D = xpcfComponentManager->resolve<solver::pose::I3DTransformSACFinderFrom3D3D>();
	auto transform3D = xpcfComponentManager->resolve<geom::I3DTransform>();
	// init random input points
	int NB_POINTS = 30;
	int NB_OUTLIERS = 15;
	std::vector<Point3Df> pts1(NB_POINTS);
	for (auto &it : pts1) {
		Vector3f tmp = Vector3f::Random();
		it.setX(tmp(0));
		it.setY(tmp(1));
		it.setZ(tmp(2));
	}

	// init random 3D transformation
	Vector3f euler = Vector3f::Random();
	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(euler(0), Vector3f::UnitZ()) * Eigen::AngleAxisf(euler(1), Vector3f::UnitY()) * Eigen::AngleAxisf(euler(2), Vector3f::UnitX());
	Vector3f tra = Vector3f::Random();	
	Transform3Df poseToTransform;
	poseToTransform.linear() = rot;
	poseToTransform.translation() = tra;
	LOG_INFO("Transform 3D used: \n{}", poseToTransform.matrix());

	// apply 3D transformation to input points
	std::vector<Point3Df> pts2;	
	std::vector<bool> bInliers(NB_POINTS, true);
	transform3D->transform(pts1, poseToTransform, pts2);
	for (int i = 0; i < NB_OUTLIERS; ++i) {
		int index = rand() % NB_POINTS;
		pts2[index].setZ(pts2[index].getZ() + 0.2 + (float)rand() / (RAND_MAX));
		bInliers[index] = false;		
	}
	LOG_INFO("Inliers indices: ");
	for (int i = 0; i < NB_POINTS; ++i)
		if (bInliers[i])
			std::cout << i << " ";
	std::cout << "\n\n";

	// find transformation
	Transform3Df pose;
	std::vector<int> inliers;
	if (estimator3D->estimate(pts1, pts2, pose, inliers) == FrameworkReturnCode::_ERROR_) {
		LOG_ERROR("Cannot find 3D transformation");
		return 0;
	}

	// display results	
	LOG_INFO("Transform 3D found: \n{}", pose.matrix());
	LOG_INFO("Inliers indices found: ");
	for (auto &it : inliers)
		std::cout << it << " ";
	std::cout << std::endl;

    return 0;
}





