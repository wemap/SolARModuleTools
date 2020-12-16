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

#ifndef SOLARFIDUCIALMARKERPOSEESTIMATOR_H
#define SOLARFIDUCIALMARKERPOSEESTIMATOR_H
#include "api/solver/pose/IFiducialMarkerPose.h"
#include "api/input/files/IMarker2DSquaredBinary.h"
#include "api/image/IImageFilter.h"
#include "api/image/IImageConvertor.h"
#include "api/features/IContoursExtractor.h"
#include "api/features/IContoursFilter.h"
#include "api/image/IPerspectiveController.h"
#include "api/features/IDescriptorsExtractorSBPattern.h"
#include "api/features/ISBPatternReIndexer.h"
#include "api/geom/IImage2WorldMapper.h"
#include "api/solver/pose/I3DTransformFinderFrom2D3D.h"
#include "datastructure/Image.h"
#include "datastructure/FiducialMarker.h"
#include "SolARToolsAPI.h"
#include "xpcf/component/ConfigurableBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
* @class SolARFiducialMarkerPoseEstimator
* @brief <B>Estimate camera pose based on a fiducial marker.</B>
* <TT>UUID: cddd23c4-da4e-4c5c-b3f9-7d095d097c97</TT>
*
*/

class SOLAR_TOOLS_EXPORT_API SolARFiducialMarkerPoseEstimator : public org::bcom::xpcf::ConfigurableBase,
	public api::solver::pose::IFiducialMarkerPose
{
public:
	///@brief SolAR3DTransformEstimationFrom3D3D constructor;
	SolARFiducialMarkerPoseEstimator();
	///@brief SolAR3DTransformEstimationFrom3D3D destructor;
    ~SolARFiducialMarkerPoseEstimator() = default;
	/// @brief this method is used to set intrinsic parameters and distorsion of the camera
        /// @param[in] Camera calibration matrix parameters.
        /// @param[in] Camera distorsion parameters.
	void setCameraParameters(const datastructure::CamCalibration & intrinsicParams, const datastructure::CamDistortion & distorsionParams) override;

    /// @brief this method is used to set the fiducial marker
    /// @param[in] Fiducial marker.
    void setMarker(const SRef<api::input::files::IMarker2DSquaredBinary> marker) override;

    /// @brief this method is used to set the fiducial marker
    /// @param[in] Fiducial marker.
    void setMarker(const SRef<datastructure::FiducialMarker> marker) override;

    /// @brief Estimates camera pose based on a fiducial marker.
	/// @param[in] image: input image.
	/// @param[out] pose: camera pose.
	/// @return FrameworkReturnCode::_SUCCESS if the estimation succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode estimate(const SRef<datastructure::Image> image, datastructure::Transform3Df & pose) override;

	void unloadComponent() override final;

private:
	datastructure::CamCalibration										m_camMatrix;
	datastructure::CamDistortion										m_camDistortion;
	SRef<api::input::files::IMarker2DSquaredBinary>		m_binaryMarker;
	SRef<api::image::IImageFilter>						m_imageFilterBinary;
	SRef<api::image::IImageConvertor>					m_imageConvertor;
	SRef<api::features::IContoursExtractor>				m_contoursExtractor;
	SRef<api::features::IContoursFilter>				m_contoursFilter;
	SRef<api::image::IPerspectiveController>			m_perspectiveController;
	SRef<api::features::IDescriptorsExtractorSBPattern>	m_patternDescriptorExtractor;
	SRef<api::features::IDescriptorMatcher>				m_patternMatcher;
	SRef<api::features::ISBPatternReIndexer>			m_patternReIndexer;
	SRef<api::geom::IImage2WorldMapper>					m_img2worldMapper;
	SRef<api::solver::pose::I3DTransformFinderFrom2D3D>	m_pnp;
	SRef<datastructure::DescriptorBuffer>				m_markerPatternDescriptor;
	int													m_nbThreshold = 3;
	int													m_minThreshold = -1;
	int													m_maxThreshold = 220;
};

}
}
}

#endif // SOLARFIDUCIALMARKERPOSEESTIMATOR_H
