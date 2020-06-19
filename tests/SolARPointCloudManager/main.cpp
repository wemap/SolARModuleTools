// Copyright (C) 2017-2019 Jonathan Müller <jonathanmueller.dev@gmail.com>
// This file is subject to the license terms in the LICENSE file
// found in the top-level directory of this distribution.

#include <boost/log/core.hpp>
#include "xpcf/xpcf.h"
#include <iostream>
#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include <api/storage/IPointCloudManager.h>
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;
using namespace SolAR;
using namespace SolAR::datastructure;
using namespace SolAR::api;

int main(int argc, char* argv[])
{
#if NDEBUG
	boost::log::core::get()->set_logging_enabled(false);
#endif
	LOG_ADD_LOG_TO_CONSOLE();
	SRef<xpcf::IComponentManager> xpcfComponentManager = xpcf::getComponentManagerInstance();

	if (xpcfComponentManager->load("testSolARPointCloudManager_conf.xml") != org::bcom::xpcf::_SUCCESS)
	{
		std::cerr << "Failed to load the configuration file testSolARPointCloudManager_conf.xml" << std::endl;
		return -1;
	}
    auto pointCloud = xpcfComponentManager->resolve<SolAR::api::storage::IPointCloudManager>();

	// create a covisibility graph
	std::string fileName = "pointcloud.txt";
	LOG_INFO("Load the point cloud from {}", fileName);
	if (pointCloud->loadFromFile(fileName) == FrameworkReturnCode::_ERROR_) {
		LOG_INFO("This file doesn't exist. Create a new point cloud");
		for (int i = 0; i < 3; i++) {
			std::map<uint32_t, uint32_t> visibility;
			visibility[i] = i;
			SRef<DescriptorBuffer> descriptor = xpcf::utils::make_shared<DescriptorBuffer>(DescriptorType::AKAZE, i + 1);
			SRef<CloudPoint> point = xpcf::utils::make_shared<CloudPoint>(0.1 * i, 0.1 * i, 0.1 * i, 0.2 * i, 0.2 * i, 0.2 * i, 0.3 * i, 0.3 * i, 0.3 * i, 0.4 * i, visibility, descriptor);
			pointCloud->addPoint(point);
		}
		pointCloud->saveToFile(fileName);
	}
	else {
		LOG_INFO("Load done");
	}
	
	for (int i = 0; i < 3; i++) {
		SRef<CloudPoint> point;
		pointCloud->getPoint(i, point);		
		LOG_INFO("Information of point {}", point->getId());
		std::cout << "Coordinate: " << point->getX() << " " << point->getY() << " " << point->getZ() << std::endl;
		std::cout << "Color: " << point->getR() << " " << point->getG() << " " << point->getB() << std::endl;
		std::cout << "Normal: " << point->getViewDirection()[0] << " " << point->getViewDirection()[1] << " " << point->getViewDirection()[2] << std::endl;
		const std::map<uint32_t, uint32_t> &visibility = point->getVisibility();
		std::cout << "Visibility: " << std::endl;
		for (const auto & it : visibility)
			std::cout << "\t" << it.first << " - " << it.second << std::endl;
		const SRef<DescriptorBuffer> &descriptor = point->getDescriptor();
		std::cout << "Nb descriptor: " << descriptor->getNbDescriptors() << "    Nb elements: " << descriptor->getNbElements() << "   Descriptor type: " << 
			descriptor->getDescriptorType() << "  Descriptor data type: " << descriptor->getDescriptorDataType() << std::endl;
		std::cout << "Primitive info: " << std::endl;
		std::cout << "Confidence: " << point->getConfidence() << std::endl;
		std::cout << "NbUsedTime: " << point->getUsedTime() << std::endl;
		std::cout << "SemanticId: " << point->getSemanticId() << std::endl;
		std::time_t time = std::chrono::system_clock::to_time_t(point->getLastUpdateTime());
		std::cout << "LastUpdateTime: " << std::ctime(&time) << std::endl;
	}

    return 0;
}
