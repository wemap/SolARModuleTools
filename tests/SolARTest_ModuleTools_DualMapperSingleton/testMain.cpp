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

#include <iostream>

#include <xpcf/api/IComponentManager.h>
#include <xpcf/core/helpers.h>
#include <api/solver/map/IMapper.h>
#include "KeyFrameRetrieverMock.h"

namespace xpcf = org::bcom::xpcf;


// print error message
void print_error(const std::string& msg)
{
    std::cerr << msg << '\n';
}

int main(int argc, char* argv[])
{
    SRef<xpcf::IComponentManager> cmpMgr = xpcf::getComponentManagerInstance();
    cmpMgr->load("SolARTest_ModuleTools_DualMapperSingleton_conf.xml");
    cmpMgr->bindLocal<SolAR::api::reloc::IKeyframeRetriever,SolAR::MODULES::TOOLS::KeyFrameRetrieverMock,xpcf::Singleton>();
    auto mapper = cmpMgr->resolve<SolAR::api::solver::map::IMapper>();
    auto otherMapper = cmpMgr->resolve<SolAR::api::solver::map::IMapper>();
    SRef<SolAR::api::storage::IKeyframesManager> keyFramesMgr, otherKeyFramesMgr;
    mapper->getKeyframesManager(keyFramesMgr);
    if (keyFramesMgr) {
        std::cout<<"IKeyframesManager instance loaded in mapper"<<std::endl;
    }

    SRef<SolAR::api::storage::IPointCloudManager> pointCloudMgr, otherPointCloudMgr;
    mapper->getPointCloudManager(pointCloudMgr);
    if (pointCloudMgr) {
        std::cout<<"IPointCloudManager instance loaded in mapper"<<std::endl;
    }

    SRef<SolAR::api::reloc::IKeyframeRetriever> keyFrameRetriever, otherKeyFrameRetriever;
    mapper->getKeyframeRetriever(keyFrameRetriever);
    if (keyFrameRetriever) {
        std::cout<<"IKeyframeRetriever instance loaded in mapper"<<std::endl;
    }

    otherMapper->getKeyframesManager(otherKeyFramesMgr);
    otherMapper->getPointCloudManager(otherPointCloudMgr);
    otherMapper->getKeyframeRetriever(otherKeyFrameRetriever);
    if  (otherKeyFrameRetriever == keyFrameRetriever) {
        std::cout<<"IKeyframeRetriever instance is a singleton"<<std::endl;
    }
    else {
        std::cout<<"IKeyframeRetriever is a dedicated instance for each new component"<<std::endl;
    }
    if (otherPointCloudMgr == pointCloudMgr) {
        std::cout<<"IPointCloudManager instance is a singleton"<<std::endl;
    }
    else {
        std::cout<<"IPointCloudManager is a dedicated instance for each new component"<<std::endl;
    }
    if (otherKeyFramesMgr == keyFramesMgr) {
        std::cout<<"IKeyframesManager instance is a singleton"<<std::endl;
    }
    else {
        std::cout<<"IKeyframesManager is a dedicated instance for each new component"<<std::endl;
    }
    std::cout<<"memory mapping :";
    std::cout<<"--> IKeyframeRetriever first instance address = "<<&*keyFrameRetriever<<std::endl;
    std::cout<<"--> IKeyframeRetriever second instance address = "<<&*otherKeyFrameRetriever<<std::endl;
    std::cout<<"--> IPointCloudManager first instance address = "<<&*pointCloudMgr<<std::endl;
    std::cout<<"--> IPointCloudManager second instance address = "<<&*otherPointCloudMgr<<std::endl;
    std::cout<<"--> IKeyframesManager first instance address = "<<&*keyFramesMgr<<std::endl;
    std::cout<<"--> IKeyframesManager second instance address = "<<&*otherKeyFramesMgr<<std::endl;

    return 0;
}
