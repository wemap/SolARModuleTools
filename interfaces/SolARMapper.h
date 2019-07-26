#ifndef SOLARMAPPER_H
#define SOLARMAPPER_H


#include "api/solver/map/IMapper.h"
#include "xpcf/component/ComponentBase.h"
#include <vector>
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

    ~SolARMapper() = default;

    /// @brief  Updates a point map with new triangulated 3D points.
    /// @param[in, out] map The 3D point map to update.
    /// @param[in] newKeyframe The new keyframe used for to triangulate the 3D points to add.
    /// @param[in] newCloud The new triangulated point to add to the map. Must be empty to initialize the map with the first keyframe.
    /// @param[in] newPointsMatches The keypoints matches for which the corresponding triangulated 3D point was not already in the map. Must be empty for the initialization of the map with the first keyframe or for the second call with the second keyframe and the first set of triangulated points.
    /// @param[in] existingPointsMatches The keypoints matches for which the corresponding triangulated 3D point is already in the map. Must be empty for the initialization of the map with the first keyframe or for the second call with the second keyframe and the first set of triangulated points.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode update (SRef<Map>& map,
                                SRef<Keyframe> newKeyframe,
                                const std::vector<SRef<CloudPoint>>& newCloud = {},
                                const std::vector<DescriptorMatch>& newPointsMatches = {},
                                const std::vector<DescriptorMatch>& existingPointsMatches = {}) override;

    /// @brief return all the keyframes of the map.
    /// @return the keyframes of the map.
    std::vector<SRef<Keyframe>> getKeyframes() override;

    /// @brief return the current map.
    /// @return the current map.
    SRef<Map> getMap() override;

    void unloadComponent () override final;

private:
    std::vector<SRef<Keyframe>>m_kframes;
    std::map<std::pair<int, int>, std::vector<DescriptorMatch> > m_gmatches;
    SRef<Map> m_map;

};
}
}
}

#endif // SOLARMAPPER_H
