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

#include "api/source/ISourceImage.h"
#include "xpcf/component/ConfigurableBase.h"

#include "datastructure/Image.h"

#include "SolARToolsAPI.h"

#include <mutex>

namespace SolAR {
using namespace datastructure;
using namespace api::source;
namespace MODULES {
namespace TOOLS {

/**
 * @class SolARBasicSource
 * @brief <B>Feeds a pipeline with an external image.</B>
 * <TT>UUID: 1e43cda9-7850-4a8a-a32b-f3f31ea94902</TT>
 *
 */

class SOLAR_TOOLS_EXPORT_API SolARBasicSource : public org::bcom::xpcf::ConfigurableBase,
    public api::source::ISourceImage
{
public:
    SolARBasicSource();
    ~SolARBasicSource() = default;

    /// @brief Set a new image coming from a third party.
    /// @param[in,out] image
    SourceReturnCode setInputTexture( void* sourceTexturehandle, int width, int height ) override;

    /// @brief Get a pointer to the texture buffer to update it with the new image when required.
    /// @param[in,out] image
    SourceReturnCode getNextImage(SRef<Image> & image) override;

    void unloadComponent () override final;

private:
    SRef<Image> m_image;

    bool m_newImage;

    std::mutex m_mutex;
};

}
}
}
