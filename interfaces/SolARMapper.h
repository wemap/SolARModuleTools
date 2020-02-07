#ifndef SOLARMAPPER_H
#define SOLARMAPPER_H


#include "api/solver/map/IMapper.h"

#include "xpcf/component/ComponentBase.h"
#include <vector>
#include <set>
#include "SolARToolsAPI.h"

#include <string>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARMapper
 * @brief <B>Updates a point map with new triangulated 3D points.</B>
 * <TT>UUID: 8e3c926a-0861-46f7-80b2-8abb5576692c</TT>
 *
 */

/**
* @class SolARMapper
* @brief Store the 3D Map and keyframes
*/
class SOLAR_TOOLS_EXPORT_API SolARMapper : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::IMapper {
public:
    SolARMapper();

    ~SolARMapper() override = default;
    /// @brief  Updates a point map with new triangulated 3D points.
    /// i.e. the new triangulated map points at the insertion of a new keyframe.
    /// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),   
    /// @param[in, out] map The 3D point map to update.
    /// @param[in] newKeyframe The new keyframe used for to triangulate the 3D points to add.
    /// @param[in] newCloud The new triangulated point to add to the map. Must be empty to initialize the map with the first keyframe.
    /// @param[in] newPointsMatches The keypoints matches for which the corresponding triangulated 3D point was not already in the map. Must be empty for the initialization of the map with the first keyframe or for the second call with the second keyframe and the first set of triangulated points.
    /// @param[in] existingPointsMatches The keypoints matches for which the corresponding triangulated 3D point is already in the map. Must be empty for the initialization of the map with the first keyframe or for the second call with the second keyframe and the first set of triangulated points.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    virtual FrameworkReturnCode update (SRef<Map> & map,
                                        SRef<Keyframe> & newKeyframe,
                                        const std::vector<CloudPoint> & newCloud,
                                        const std::vector<DescriptorMatch> & newPointsMatches,
                                        const std::vector<DescriptorMatch> & existingPointsMatches) override;

	/// @brief update the current map with the new triangulated map points at the insertion of a new keyframe.
   /// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),
   /// @param[in,out] map current constructed map.
   /// @param[in,out] neyKeyframe current new keyframe to insert.
   /// @param[in] newCloud new triangulated 3D points
   /// @param[in] newPointMatches a set of tuple contains information of matches corresponding newCloudPoint. 
   ///			  The first value is the idx of keypoint in the new keyframe. The remaining values are idx of keyframe and its keypoint.
   /// @param[in] existingPointMatches new detected matches from the reference keyframe and current frame.
   /// @return FrameworkReturnCode::_SUCCESS if the map updating succeed, else FrameworkReturnCode::_ERROR_
	virtual FrameworkReturnCode update(	SRef<Map> & map,
										SRef<Keyframe> & newKeyframe,
										const std::vector<CloudPoint> & newCloud,
										const std::vector<std::tuple<unsigned int, int, unsigned int>> &newPointMatches) override;



	/// @brief update the current map/keyframes(poses)with corrected map/keyframes(poses).
	/// i.e. the new triangulated map points at the insertion of a new keyframe.
	/// minArg(pts3ds,intrinsics,extrinsics) = MIN_cam_i(MIN_3d_j(pts2d_j - reproje(pt3ds_j,intrinsics_i,extrinsics_i)),   
	/// @param[in, out] map The 3D point map to update.
	virtual FrameworkReturnCode update(const std::vector<Transform3Df> & correctedPoses,
									   const std::vector<CloudPoint> & correctedCloud) override;


    /// @brief return all the keyframes of the map.
    /// @return the keyframes of the map.
	virtual const std::vector<SRef<Keyframe>> &getKeyframes() override 
	{
		std::unique_lock<std::mutex> lock(m_mutex);
		return m_kframes; 
	};

	/// @brief return a keyframe
	/// @param[in] Index of the keyframe
	virtual SRef<Keyframe> &getKeyframe(int index) override 
	{ 
		std::unique_lock<std::mutex> lock(m_mutex);
		return m_kframes[index]; 
	};

	/// @brief get local map from reference keyframe and its neighbors
	virtual void getLocalMap(SRef<Keyframe> refKF, std::vector<CloudPoint> &localCloudPoints) override;
	virtual SRef<Map> getGlobalMap() override;

	/// @brief get index of cloud point in local map from reference keyframe and its neighbors
	virtual void getLocalMapIndex(SRef<Keyframe> refKF, std::vector<unsigned int> &idxLocalCloudPoints) override;    

	/// @brief update connections between new keyframe and neighboring keyframes.
	/// @param[in] a new keyframe.
	/// @param[in] the minimum number of common point to accept a connection.
	virtual void updateNeighborConnections(SRef<Keyframe> &newKeyframe, int minDis) override;

	/// @brief return the current map.
	/// @return the current map.
	inline SRef<Map> getMap() {
		std::unique_lock<std::mutex> lock(m_mutex);
		return m_map;
	};

    void unloadComponent () override final;	

private:
    std::vector<SRef<Keyframe>> m_kframes;
    std::map<std::pair<int, int>, std::vector<DescriptorMatch> > m_gmatches;
    SRef<Map> m_map;
	std::mutex m_mutex;
};
}
}
}

#endif // SOLARMAPPER_H
