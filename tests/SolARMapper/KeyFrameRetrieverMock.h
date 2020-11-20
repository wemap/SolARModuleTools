/**
 * @copyright Copyright (c) 2019 B-com http://www.b-com.com/
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
 *
 * @author Loïc Touraine
 *
 * @file
 * @brief description of file
 * @date 2019-11-15
 */

#ifndef KEYFRAMERETRIEVERMOCK_H
#define KEYFRAMERETRIEVERMOCK_H
#include <xpcf/component/ConfigurableBase.h>
#include <api/reloc/IKeyframeRetriever.h>

namespace SolAR {
namespace MODULES {
namespace TOOLS {

class KeyFrameRetrieverMock : public org::bcom::xpcf::ConfigurableBase, virtual public SolAR::api::reloc::IKeyframeRetriever
{
public:
    KeyFrameRetrieverMock();
    ~KeyFrameRetrieverMock() override;
    void unloadComponent () override;
    org::bcom::xpcf::XPCFErrorCode onConfigured() override;

    /// @brief Add a keyframe to the retrieval model
    /// @param[in] keyframe: the keyframe to add to the retrieval model
    /// @return FrameworkReturnCode::_SUCCESS if the keyfram adding succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode addKeyframe(const SRef<Keyframe>& keyframe) override;

    /// @brief Suppress a keyframe from the retrieval model
    /// @param[in] keyframe_id: the keyframe to supress from the retrieval model
    /// @return FrameworkReturnCode::_SUCCESS if the keyfram adding succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode suppressKeyframe(uint32_t keyframe_id) override;


    /// @brief Retrieve a set of keyframes close to the frame pass in input.
    /// @param[in] frame: the frame for which we want to retrieve close keyframes.
    /// @param[out] retKeyframes_id: a set of keyframe ids which are close to the frame pass in input
    /// @return FrameworkReturnCode::_SUCCESS if the retrieve succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode retrieve(const SRef<Frame>& frame, std::vector<uint32_t> &retKeyframes_id) override;

    /// @brief Retrieve a set of keyframes close to the frame pass in input.
    /// @param[in] frame: the frame for which we want to retrieve close keyframes.
    /// @param[in] canKeyframes_id: a set includes id of keyframe candidates
    /// @param[out] retKeyframes_id: a set of keyframe ids which are close to the frame pass in input
    /// @return FrameworkReturnCode::_SUCCESS if the retrieve succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode retrieve(const SRef<Frame>& frame, std::set<unsigned int> &canKeyframes_id, std::vector<uint32_t> & retKeyframes_id) override;

    /// @brief This method allows to save the keyframe feature to the external file
    /// @param[out] the file name
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode saveToFile(const std::string& file) override;

    /// @brief This method allows to load the keyframe feature from the external file
    /// @param[in] the file name
    /// @return FrameworkReturnCode::_SUCCESS_ if the suppression succeed, else FrameworkReturnCode::_ERROR.
    FrameworkReturnCode loadFromFile(const std::string& file) override;


    /// @brief Match a frame with a keyframe
    /// @param[in] frame: the frame to match
    /// @param[in] keyframe_id: id of keyframe to match
    /// @param[out] matches: a set of matches between frame and keyframe
    /// @return FrameworkReturnCode::_SUCCESS if the retrieve succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode match(const SRef<Frame>& frame, const SRef<Keyframe>& keyframe, std::vector<DescriptorMatch> &matches) override;

    /// @brief Match a set of descriptors with a keyframe
    /// @param[in] indexDescriptors: index of descriptors to match.
    /// @param[in] descriptors: a descriptor buffer contains all descriptors
    /// @param[in] keyframe_id: id of keyframe to match
    /// @param[out] matches: a set of matches between frame and keyframe
    /// @return FrameworkReturnCode::_SUCCESS if the retrieve succeed, else FrameworkReturnCode::_ERROR_
    FrameworkReturnCode match(const std::vector<int> &indexDescriptors, const SRef<DescriptorBuffer> &descriptors, const SRef<Keyframe> &keyframe, std::vector<DescriptorMatch> &matches) override;

private:

    /// @brief path to the vocabulary file
    std::string m_VOCPath   = "";

    /// @brief the threshold above which keyframes are considered valid
    float m_threshold       = 0;

    /// @brief level stored for BoW2
    int	m_level				= 1;

    /// @brief For each node at level m_level stores a set of frames that contain it
    std::map<uint32_t, std::set<uint32_t>> m_invertedIndexKfs;

    /// @brief distance ratio used to keep good matches.
    float m_distanceRatio = 0.7;

    /// @brief distance max used to keep good matches.
    float m_distanceMax = 100;

};

}}}

template <> struct org::bcom::xpcf::ComponentTraits<SolAR::MODULES::TOOLS::KeyFrameRetrieverMock>
{
    static constexpr const char * UUID = "{C707E467-2FB7-443C-972D-79339BE28B57}";
    static constexpr const char * NAME = "KeyFrameRetrieverMock";
    static constexpr const char * DESCRIPTION = "KeyFrameRetrieverMock implements IKeyFrameRetriever interface";
};


#endif
