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

#ifndef SOLARSBPATTERNREINDEXER_H
#define SOLARSBPATTERNREINDEXER_H

#include "api/features/ISBPatternReIndexer.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARSBPatternReIndexer
 * @brief <B>Provides both the 4 corners of a pattern in its reference coordinate system (pixels, cells, etc.) and the 4 corners in pixels of this pattern in the current image.</B>
 * <TT>UUID: a2ef5542-029e-4fce-9974-0aea14b29d6f</TT>
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARSBPatternReIndexer : public org::bcom::xpcf::ConfigurableBase,
        public api::features::ISBPatternReIndexer {
public:
    SolARSBPatternReIndexer();
    ~SolARSBPatternReIndexer() override = default;
    
    /// @brief Provides both the 4 corners of a pattern in its reference coordinate system (pixels, cells, etc.) and the 4 corners in pixels of this pattern in the current image.
    /// @param[in] candidateContours A set of contours of squared binary marker detected in the current image.
    /// @param[in] matches matches between the patterns of the dected contours of the pattern of teh reference squared binary marker.
    /// @param[out] patternPoints The 2D corners of the squared binary marker in the coordinate system of the marker.
    /// @param[out] imagePoints The 2D corners of the squared binary marker in the coordinate system of the current image.
    /// @return FrameworkReturnCode::_SUCCESS if sucessful, eiher FrameworkRetunrnCode::_ERROR_.
    FrameworkReturnCode reindex(const std::vector<datastructure::Contour2Df> & candidateContours,
                                const std::vector<datastructure::DescriptorMatch> & matches,
                                std::vector<datastructure::Point2Df> & patternPoints,
                                std::vector<datastructure::Point2Df> & imagePoints) override;

    void unloadComponent () override final;

private:
   int m_sbPatternSize;
};

}
}
}  // end of namespace Solar

#endif // SOLARSBPATTERNREINDEXER_H
