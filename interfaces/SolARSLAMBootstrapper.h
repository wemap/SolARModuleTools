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

#ifndef SOLARSLAMBOOTSTRAPPER_H
#define SOLARSLAMBOOTSTRAPPER_H
#include "api/slam/IBootstrapper.h"
#include "datastructure/Image.h"
#include "api/solver/map/IMapper.h"
#include "api/features/IKeypointDetector.h"
#include "api/features/IDescriptorsExtractor.h"
#include "api/features/IDescriptorMatcher.h"
#include "api/features/IMatchesFilter.h"
#include "api/solver/map/ITriangulator.h"
#include "api/solver/map/IMapFilter.h"
#include "api/solver/map/IKeyframeSelector.h"
#include "api/display/IMatchesOverlay.h"
#include "api/solver/pose/I3DTransformFinderFrom2D2D.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARSLAMBootstrapper
* @brief <B>Initialization SLAM using an image stream of a camera.</B>
* <TT>UUID: 8f43eed0-1a2e-4c47-83f0-8dd5b259cdb0</TT>
*
*/

class SOLAR_TOOLS_EXPORT_API SolARSLAMBootstrapper : public org::bcom::xpcf::ConfigurableBase,
	public api::slam::IBootstrapper
{
public:
	///@brief SolAR3DTransformEstimationFrom3D3D constructor;
	SolARSLAMBootstrapper();
	///@brief SolAR3DTransformEstimationFrom3D3D destructor;
	~SolARSLAMBootstrapper() = default;
	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
	/// @param[in] Camera calibration matrix parameters.
	/// @param[in] Camera distorsion parameters.
	void setCameraParameters(const datastructure::CamCalibration & intrinsicParams, const datastructure::CamDistortion & distorsionParams) override;

	/// @brief This method uses images to boostrap
	/// @param[in] image: input image to process
	/// @param[out] view: output image to visualize
	/// @param[in] pose: the pose of the input image
	/// @return FrameworkReturnCode::_SUCCESS_ if initialization succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode process(const SRef<datastructure::Image> image, SRef<datastructure::Image> & view, const datastructure::Transform3Df & pose = datastructure::Transform3Df::Identity()) override;

	void unloadComponent() override final;

private:
	/// bootstrap uses marker
	FrameworkReturnCode initFiducialMarker();
	/// bootstrap doesn't use marker
	FrameworkReturnCode initMarkerLess();

private:
	int													m_hasPose = 1;
	int													m_nbMinInitPointCloud = 50;
    float												m_angleThres = 0.1f;
	bool												m_bootstrapOk = false;
	bool												m_initKeyframe1 = false;
	SRef<datastructure::Keyframe>										m_keyframe1, m_keyframe2;
	datastructure::CamCalibration										m_camMatrix;
	datastructure::CamDistortion										m_camDistortion;
	SRef<api::solver::map::IMapper>						m_mapper;
	SRef<api::features::IKeypointDetector>				m_keypointsDetector;
	SRef<api::features::IDescriptorsExtractor>			m_descriptorExtractor;
	SRef<api::features::IDescriptorMatcher>				m_matcher;
	SRef<api::features::IMatchesFilter>					m_matchesFilter;
	SRef<api::solver::map::ITriangulator>				m_triangulator;
	SRef<api::solver::map::IMapFilter>					m_mapFilter;
	SRef<api::solver::map::IKeyframeSelector>			m_keyframeSelector;
	SRef<api::solver::pose::I3DTransformFinderFrom2D2D>	m_poseFinderFrom2D2D;
	SRef<api::display::IMatchesOverlay>					m_matchesOverlay;
};

}
}
}

#endif // SOLARSLAMBOOTSTRAPPER_H
