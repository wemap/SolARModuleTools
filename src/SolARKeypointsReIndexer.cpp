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

#include "SolARKeypointsReIndexer.h"
#include "ComponentFactory.h"

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

    SolARKeypointsReIndexer::SolARKeypointsReIndexer():ComponentBase(xpcf::toUUID<SolARKeypointsReIndexer>())
    {
        addInterface<api::features::IKeypointsReIndexer>(this);
    }

  FrameworkReturnCode SolARKeypointsReIndexer::reindex(const std::vector<SRef<Keypoint>>& refKeypoints, const std::vector<SRef<Keypoint>>& imgKeypoints, std::vector<DescriptorMatch>& matches, std::vector<SRef<Point2Df>>& matchedRefKeypoints, std::vector<SRef<Point2Df>>& matchedImgKeypoints)
    {
        matchedRefKeypoints.clear();
        matchedImgKeypoints.clear();

       for( int i = 0; i < matches.size(); i++ )
       {
            matchedRefKeypoints.push_back(xpcf::utils::make_shared<Point2Df>(refKeypoints[ matches[i].getIndexInDescriptorA()]->getX(),refKeypoints[ matches[i].getIndexInDescriptorA()]->getY()));
            matchedImgKeypoints.push_back(xpcf::utils::make_shared<Point2Df>(imgKeypoints[ matches[i].getIndexInDescriptorB()]->getX(),imgKeypoints[ matches[i].getIndexInDescriptorB()]->getY()));
       }
       return FrameworkReturnCode::_SUCCESS;
    }

}
}
}  // end of namespace Solar
