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
using namespace api::storage;
using namespace api::reloc;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARMapper
 * @brief <B>Allow to manage all components of a map.</B>
 * <TT>UUID: 8e3c926a-0861-46f7-80b2-8abb5576692c</TT>
 *
 */

/**
* @class SolARMapper
* @brief Store all components of a map
*/
class SOLAR_TOOLS_EXPORT_API SolARMapper : public org::bcom::xpcf::ComponentBase,
    public api::solver::map::IMapper {
public:
    SolARMapper();

    ~SolARMapper() override = default;

	/// @brief Set identification component.
   /// @param[in] an identification instance
   /// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode setIdentification(SRef<Identification> &identification) override;

	/// @brief Get identification component.
	/// @param[out] an identification instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getIdentification(SRef<Identification> &identification) override;

	/// @brief Set coordinate system component.
	/// @param[in] a coordinate system instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode setCoordinateSystem(SRef<CoordinateSystem> &coordinateSystem) override;

	/// @brief Get coordinate system component.
	/// @param[out] a coordinate system instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getCoordinateSystem(SRef<CoordinateSystem> &coordinateSystem) override;

	/// @brief Set point cloud component.
	/// @param[in] a point cloud instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode setPointCloudManager(SRef<IPointCloudManager> &pointCloudManager) override;

	/// @brief Get point cloud component.
	/// @param[out] a point cloud instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getPointCloudManager(SRef<IPointCloudManager> &pointCloudManager) override;

	/// @brief Set keyframes manager component.
	/// @param[in] a keyframes manager instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode setKeyframesManager(SRef<IKeyframesManager> &keyframesManager) override;

	/// @brief Get keyframes manager component.
	/// @param[out] a keyframes manager instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getKeyframesManager(SRef<IKeyframesManager> &keyframesManager) override;

	/// @brief Set covisibility graph component.
	/// @param[in] a covisibility graph instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode setCovisibilityGraph(SRef<ICovisibilityGraph> &covisibilityGraph) override;

	/// @brief Get covisibility graph component.
	/// @param[out] a covisibility graph instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getCovisibilityGraph(SRef<ICovisibilityGraph> &covisibilityGraph) override;

	/// @brief Set keyframe retriever component.
	/// @param[in] a keyframe retriever instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode setKeyframeRetriever(SRef<IKeyframeRetriever> &keyframeRetriever) override;

	/// @brief Get keyframe retriever component.
	/// @param[out] a keyframe retriever instance
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getKeyframeRetriever(SRef<IKeyframeRetriever> &keyframeRetriever) override;

	/// @brief Save the map to the external file
	 /// @param[out] the file name
	 /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode saveToFile(std::string file) override;

	/// @brief Load the map from the external file
	/// @param[in] the file name
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile(std::string file) override;

    void unloadComponent () override final;	

private:
	SRef<Identification>		m_identification;
	SRef<CoordinateSystem>		m_coordinateSystem;
	SRef<IPointCloudManager>	m_pointCloudManager;
	SRef<IKeyframesManager>		m_keyframesManager;
	SRef<ICovisibilityGraph>	m_covisibilityGraph;
	SRef<IKeyframeRetriever>	m_keyframeRetriever;
	std::mutex					m_mutex;
};
}
}
}

#endif // SOLARMAPPER_H
