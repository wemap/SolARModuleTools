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

#ifndef SOLARSLAMMAPPING_H
#define SOLARSLAMMAPPING_H
#include "api/slam/IMapping.h"
#include "api/solver/map/IMapper.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/storage/ICovisibilityGraph.h"
#include "api/storage/IKeyframesManager.h"
#include "api/storage/IPointCloudManager.h"
#include "api/solver/map/IBundler.h"
#include "api/reloc/IKeyframeRetriever.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/geom/IProject.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/solver/pose/I2D3DCorrespondencesFinder.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
* @class SolARSLAMMapping
* @brief <B> SLAM mapping.</B>
* <TT>UUID: c276bcb1-2ac8-42f2-806d-d4fe0ce7d4be</TT>
*
*/

class SOLAR_TOOLS_EXPORT_API SolARSLAMMapping : public org::bcom::xpcf::ConfigurableBase,
	public api::slam::IMapping
{
public:
	///@brief SolARSLAMMapping constructor;
	SolARSLAMMapping();

	///@brief SolARSLAMMapping destructor;
	~SolARSLAMMapping() = default;

	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const CamCalibration & intrinsicParams, const CamDistortion & distorsionParams) override;

	/// @brief this method is used to process mapping task.
	/// @param[in] frame: the input frame.
	/// @param[in] keyframe: new keyframe or new reference keyframe found.
	FrameworkReturnCode process(const SRef<Frame> & frame, SRef<Keyframe> & keyframe) override;

	void unloadComponent() override final;

private:
	SRef<Keyframe> processNewKeyframe(const SRef<Frame> &frame);
	bool checkNeedNewKeyframeInLocalMap(const SRef<Frame> &frame);
	void updateAssociateCloudPoint(const SRef<Keyframe> &keyframe);
	void findMatchesAndTriangulation(const SRef<Keyframe> & keyframe, const std::vector<uint32_t> &idxBestNeighborKfs, std::vector<SRef<CloudPoint>> &cloudPoint);
	void fuseCloudPoint(const SRef<Keyframe> &keyframe, const std::vector<uint32_t> &idxNeigborKfs, std::vector<SRef<CloudPoint>> &newCloudPoint);
	void cloudPointsCulling(const SRef<Keyframe> &keyframe);

private:
	float												m_minWeightNeighbor = 1.f;
	int													m_minTrackedPoints = 100;
	int													m_maxNbNeighborKfs = 5;
	SRef<Keyframe>										m_updatedReferenceKeyframe;
	CamCalibration										m_camMatrix;
	CamDistortion										m_camDistortion;
	SRef<api::solver::map::IKeyframeSelector>			m_keyframeSelector;
	SRef<api::storage::ICovisibilityGraph>				m_covisibilityGraph;
	SRef<api::storage::IKeyframesManager>				m_keyframesManager;
	SRef<api::solver::map::IBundler>					m_bundler;
	SRef<api::reloc::IKeyframeRetriever>				m_keyframeRetriever;
	SRef<api::solver::map::IMapper>						m_mapper;
	SRef<api::storage::IPointCloudManager>				m_pointCloudManager;
	SRef<api::features::IMatchesFilter>					m_matchesFilter;
	SRef<api::solver::map::ITriangulator>				m_triangulator;
	SRef<api::solver::map::IMapFilter>					m_mapFilter;
	SRef<api::geom::IProject>							m_projector;
	SRef<api::features::IDescriptorMatcher>				m_matcher;
	SRef<api::solver::pose::I2D3DCorrespondencesFinder>	m_corr2D3DFinder;
	std::map<uint32_t, std::pair<SRef<CloudPoint>, uint32_t>>m_recentAddedCloudPoints;
};

}
}
}

#endif // SOLARSLAMMAPPING_H
