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

#ifndef SOLARKEYFRAMESELECTOR_H
#define SOLARKEYFRAMESELECTOR_H

#include "api/solver/map/IKeyframeSelector.h"
#include "xpcf/component/ConfigurableBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARKeyframeSelector
 * @brief <B>Defines if a frame can be a candidate for a keyframe.</B>
 * <TT>UUID: ad59a5ba-beb8-11e8-a355-529269fb1459</TT>
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARKeyframeSelector : public org::bcom::xpcf::ConfigurableBase,
        public api::solver::map::IKeyframeSelector {
public:

    SolARKeyframeSelector();
    ///
    ///@brief ~SolARKeyframeSelector
    ///
    virtual ~SolARKeyframeSelector() override {}

    /// @brief  Defines if a frame can be a candidate for a keyframe
    /// @param[in] frame: the frame tested to know if it could be a Keyframe
    /// @param[in] matches: the matches between the frame and its keyframe of reference.
    /// @return true if the frame can be considered as a new keyframe, false otherwise.
    virtual bool select(const SRef<Frame> & frame, const std::vector<DescriptorMatch> & matches) override;

    /// @brief  Select if a frame can be considered as a keyframe
    /// @param[in] frame: the frame tested to know if it could be a Keyframe
    /// The underlying component can use data from SolAR data storage components, based on the frame properties.
    /// @return true if the frame can be considered as a new keyframe, false otherwise.
    virtual bool select(const SRef<Frame> & frame) override;

    /// @brief  Select if a frame can be considered as a keyframe.
    /// It is based on a selection predicate and provides the mean to use any datastructure in the pipeline context to the decision algorithm.
    /// @param[in] frame: the frame tested to know if it could be a Keyframe
    /// @param[in] func: the function predicate used to test the frame.
    /// This predicate can be any lambda capturing its context (matches, point cloud, bow ...) to select the frame.
    /// @return true if the frame can be considered as a new keyframe, false otherwise.
    virtual bool select(const SRef<Frame> & frame, const std::function<bool(const SRef<Frame> &)> & func) override;

    void unloadComponent () override final;

 private:
    // Minimum number of matches for a frame to be a keyframe
    int m_minNbMatchesIsKeyframe = 50;

    // Minimum mean distance for a frame to be a keyframe
    float m_minMeanDistanceIsKeyframe = 20.0;	
};

}
}
}
#endif // SOLARKEYFRAMESELECTOR_H
