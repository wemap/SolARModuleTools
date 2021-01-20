#ifndef SOLAR3D3DCORRESPONDENCESFINDER_H
#define SOLAR3D3DCORRESPONDENCESFINDER_H
#include <vector>
#include "api/solver/pose/I3D3DCorrespondencesFinder.h"
#include "api/storage/IPointCloudManager.h"
#include "SolARToolsAPI.h"

#include "xpcf/component/ComponentBase.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {
/**
* @class SolAR3D3DCorrespondencesFinder
* @brief <B>Finds the 3D-3D correspondences from feature matches of two keyframes.</B>
* <TT>UUID: 978068ef-7f93-41ef-8e24-13419776d9c6</TT>
*
* @SolARComponentInjectablesBegin
* @SolARComponentInjectable{SolAR::api::storage::IPointCloudManager}
* @SolARComponentInjectablesEnd
*
*/
class SOLAR_TOOLS_EXPORT_API SolAR3D3DCorrespondencesFinder : public org::bcom::xpcf::ComponentBase,
	public api::solver::pose::I3D3DCorrespondencesFinder
{
public:
	///@brief SolAR3D3DCorrespondencesFinder constructor;
	SolAR3D3DCorrespondencesFinder();
	///@brief SolAR3D3DCorrespondencesFinder destructor;
	~SolAR3D3DCorrespondencesFinder()  override = default;

	/// @brief Define 3D-3D point correspondences of two keyframes based on keypoint matches.
	/// @param[in] firstKeyframe: The first keyframe.
	/// @param[in] secondKeyframe: The second keyframe.
	/// @param[in] currentMatches: The 2D matches between the current keyframe and its reference keyframe.
	/// @param[out] firstCloudPoints: The cloud points seen from the first keyframe.
	/// @param[out] secondCloudPoints: The cloud points seen from the second keyframe.
	/// @param[out] found_matches: The matches allow to define 3D-3D correspondences.
	/// @param[out] remaining_matches: The remaining matches.
	FrameworkReturnCode find(const SRef<datastructure::Keyframe> firstKeyframe,
							const SRef<datastructure::Keyframe> secondKeyframe,
							const std::vector<datastructure::DescriptorMatch> & current_matches,
							std::vector<SRef<datastructure::CloudPoint>> & firstCloudPoints,
							std::vector<SRef<datastructure::CloudPoint>> & secondCloudPoints,
							std::vector<datastructure::DescriptorMatch> & found_matches,
							std::vector<datastructure::DescriptorMatch> & remaining_matches) override;


	/// @brief Define 3D-3D point correspondences of two keyframes based on keypoint matches between different maps.
	/// @param[in] firstKeyframe: The first keyframe.
	/// @param[in] secondKeyframe: The second keyframe.
	/// @param[in] currentMatches: The 2D matches between the current keyframe and its reference keyframe.
	/// @param[out] firstCloudPointsIndices: The cloud points indices seen from the first keyframe.
	/// @param[out] secondCloudPointsIndices: The cloud points indices seen from the second keyframe.
	/// @param[out] found_matches: The matches allow to define 3D-3D correspondences.
	FrameworkReturnCode find(const SRef<datastructure::Keyframe> firstKeyframe,
							const SRef<datastructure::Keyframe> secondKeyframe,
							const std::vector<datastructure::DescriptorMatch> & current_matches,
							std::vector<uint32_t> & firstCloudPointsIndices,
							std::vector<uint32_t> & secondCloudPointsIndices,
							std::vector<datastructure::DescriptorMatch> & found_matches) override;


	void unloadComponent() override final;

private:
	SRef<api::storage::IPointCloudManager> m_pointCloudManagerG;
	SRef<api::storage::IPointCloudManager> m_pointCloudManager;
};
}
}
}
#endif // SOLAR3D3DCORRESPONDENCESFINDER_H
