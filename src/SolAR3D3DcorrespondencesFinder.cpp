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

#include "SolAR3D3DcorrespondencesFinder.h"
#include "core/Log.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR3D3DCorrespondencesFinder)

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
namespace MODULES {
namespace TOOLS {
SolAR3D3DCorrespondencesFinder::SolAR3D3DCorrespondencesFinder() :ComponentBase(xpcf::toUUID<SolAR3D3DCorrespondencesFinder>())
{
	declareInterface<api::solver::pose::I3D3DCorrespondencesFinder>(this);
	declareInjectable<IPointCloudManager>(m_pointCloudManager);
	LOG_DEBUG("SolAR3D3DCorrespondencesFinder constructor");
}

FrameworkReturnCode SolAR3D3DCorrespondencesFinder::find(const SRef<Keyframe> firstKeyframe, 
														 const SRef<Keyframe> secondKeyframe,
														 const std::vector<DescriptorMatch>& current_matches,
													 	 std::vector<SRef<CloudPoint>>& firstCloudPoints, 
														 std::vector<SRef<CloudPoint>>& secondCloudPoints,
														 std::vector<DescriptorMatch>& found_matches,
														 std::vector<DescriptorMatch>& remaining_matches)
{

	const std::map<uint32_t, uint32_t> &mapVisibility1 = firstKeyframe->getVisibility();
	const std::map<uint32_t, uint32_t> &mapVisibility2 = secondKeyframe->getVisibility();
	for (int i = 0; i < current_matches.size(); ++i) {
		SRef<CloudPoint> cloudPoint1, cloudPoint2;
		std::map<unsigned int, unsigned int>::const_iterator it_cp1 = mapVisibility1.find(current_matches[i].getIndexInDescriptorA());
		std::map<unsigned int, unsigned int>::const_iterator it_cp2 = mapVisibility2.find(current_matches[i].getIndexInDescriptorB());
		if ((it_cp1 != mapVisibility1.end()) && (m_pointCloudManager->getPoint(it_cp1->second, cloudPoint1) == FrameworkReturnCode::_SUCCESS) &&
			(it_cp2 != mapVisibility2.end()) && (m_pointCloudManager->getPoint(it_cp2->second, cloudPoint2) == FrameworkReturnCode::_SUCCESS)) {
			firstCloudPoints.push_back(cloudPoint1);
			secondCloudPoints.push_back(cloudPoint2);
			found_matches.push_back(current_matches[i]);
		}


		else {
			remaining_matches.push_back(current_matches[i]);
		}
	}
	return FrameworkReturnCode::_SUCCESS;
}


FrameworkReturnCode SolAR3D3DCorrespondencesFinder::find(const SRef<Keyframe> firstKeyframe,
														const SRef<Keyframe> secondKeyframe,
														const std::vector<DescriptorMatch> & current_matches,
														std::vector<uint32_t> & firstCloudPointsIndices,
														std::vector<uint32_t> & secondCloudPointsIndices,
														std::vector<DescriptorMatch> & found_matches) 
{

	const std::map<uint32_t, uint32_t> &mapVisibility1 = firstKeyframe->getVisibility();
	const std::map<uint32_t, uint32_t> &mapVisibility2 = secondKeyframe->getVisibility();
	for (int i = 0; i < current_matches.size(); ++i) {
		SRef<CloudPoint> cloudPoint1, cloudPoint2;
		std::map<unsigned int, unsigned int>::const_iterator it_cp1 = mapVisibility1.find(current_matches[i].getIndexInDescriptorA());
		std::map<unsigned int, unsigned int>::const_iterator it_cp2 = mapVisibility2.find(current_matches[i].getIndexInDescriptorB());
		if ((it_cp1 != mapVisibility1.end()) && (it_cp2 != mapVisibility2.end())) {
			firstCloudPointsIndices.push_back(it_cp1->second);
			secondCloudPointsIndices.push_back(it_cp2->second);
			found_matches.push_back(current_matches[i]);
		}		
	}
	return FrameworkReturnCode::_SUCCESS;
}

}
}
}
