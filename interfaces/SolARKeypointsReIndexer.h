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

#ifndef SOLARKEYPOINTSREINDEXER_H
#define SOLARKEYPOINTSREINDEXER_H

#include "api/features/IKeypointsReIndexer.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARKeypointsReIndexer
 * @brief <B>Provides two ordered set of matching keypoints from two unordered set of keypoints and their corresponding matches.</B>
 * <TT>UUID: c2836cc0-0344-4956-8959-84936fb4bcf2</TT>
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARKeypointsReIndexer : public org::bcom::xpcf::ComponentBase,
        public api::features::IKeypointsReIndexer {
public:
    SolARKeypointsReIndexer();
    ~SolARKeypointsReIndexer() = default;

    /// @brief Provides two ordered set of matching keypoints from two unordered set of keypoints and their corresponding matches.
    /// @param[in] refKeypoints The first set of unordered keypoints.
    /// @param[in] imgKeypoints The second set of unordered keypoints.
    /// @param[in] matches The matches between the two sets of unordered keypoints.
    /// @param[out] matchedRefKeypoints The ordered set of keypoints where the ith keypoint matches with the ith keypoint of the matchedImgKeypoints set.
    /// @param[out] matchedImgKeypoints The ordered set of keypoints where the ith keypoint matches with the ith keypoint of the matchedRefKeypoints set.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode reindex(const std::vector<SRef<Keypoint>>& refKeypoints, const std::vector<SRef<Keypoint>>& imgKeypoints, std::vector<DescriptorMatch>& matches, std::vector<SRef<Point2Df>>& matchedRefKeypoints, std::vector<SRef<Point2Df>>& matchedImgKeypoints) override;

    void unloadComponent () override final;

    };

}
}
}  // end of namespace Solar

#endif // SOLARKEYPOINTSREINDEXER_H
