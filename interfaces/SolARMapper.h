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
* @brief Store the 3D Map and keyframes
*/
class SOLAR_TOOLS_EXPORT_API SolARMapper : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::IMapper {
public:
    SolARMapper();

    ~SolARMapper() = default;

    virtual FrameworkReturnCode update (SRef<Map>& map,
                                        SRef<Keyframe> newKeyframe,
                                        const std::vector<SRef<CloudPoint>>& newCloud = {},
                                        const std::vector<DescriptorMatch>& newPointsMatches = {},
                                        const std::vector<DescriptorMatch>& existingPointsMatches = {}) override;

    std::vector<SRef<Keyframe>> getKeyframes() override;
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
