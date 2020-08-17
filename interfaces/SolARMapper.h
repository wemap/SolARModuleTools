#ifndef SOLARMAPPER_H
#define SOLARMAPPER_H


#include "api/solver/map/IMapper.h"

#include "xpcf/component/ComponentBase.h"
#include "xpcf/component/ConfigurableBase.h"
#include <vector>
#include <set>
#include "SolARToolsAPI.h"
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include "core/SerializationDefinitions.h"

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
class SOLAR_TOOLS_EXPORT_API SolARMapper : public org::bcom::xpcf::ConfigurableBase,
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

	/// @brief Get local point cloud seen from the keyframe and its neighbors
   /// @param[in] keyframe: the keyframe to get local point cloud
   /// @param[in] minWeightNeighbor: the weight to get keyframe neighbors
   /// @param[out] localPointCloud: the local point cloud
   /// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode getLocalPointCloud(const SRef<Keyframe> &keyframe, float minWeightNeighbor, std::vector<SRef<CloudPoint>> &localPointCloud) override;

	/// @brief Add a point cloud to mapper and update visibility of keyframes and covisibility graph
	/// @param[in] cloudPoint: the cloud point to add to the mapper
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode addCloudPoint(const SRef<CloudPoint> &cloudPoint) override;

	/// @brief Remove a point cloud from mapper and update visibility of keyframes and covisibility graph
	/// @param[in] cloudPoint: the cloud point to remove to the mapper
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode removeCloudPoint(const SRef<CloudPoint> &cloudPoint) override;

	/// @brief Remove a keyframe from mapper and update visibility of point cloud and covisibility graph
	/// @param[in] cloudPoint: the cloud point to add to the mapper
	/// @return FrameworkReturnCode::_SUCCESS if succeed, else FrameworkReturnCode::_ERROR_
	FrameworkReturnCode removeKeyframe(const SRef<Keyframe> &keyframe) override;

	/// @brief Prune cloud points and keyframes of a map
	/// @param[in] cloudPoints: the cloud points are checked to prune
	void pruning(const std::vector<SRef<CloudPoint>> &cloudPoints = {}) override;

	/// @brief Save the map to the external file
	 /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode saveToFile() override;

	/// @brief Load the map from the external file
	/// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
	FrameworkReturnCode loadFromFile() override;

    void unloadComponent () override final;	

private:
	SRef<Identification>		m_identification;
	SRef<CoordinateSystem>		m_coordinateSystem;
	SRef<IPointCloudManager>	m_pointCloudManager;
	SRef<IKeyframesManager>		m_keyframesManager;
	SRef<ICovisibilityGraph>	m_covisibilityGraph;
	SRef<IKeyframeRetriever>	m_keyframeRetriever;
	std::mutex					m_mutex;

	std::string					m_directory;
	std::string					m_identificationFileName;
	std::string					m_coordinateFileName;
	std::string					m_pcManagerFileName;
	std::string					m_kfManagerFileName;
	std::string					m_covisGraphFileName;
	std::string					m_kfRetrieverFileName;

	float						m_reprojErrorThres = 3.0;
};
}
}
}

#endif // SOLARMAPPER_H
