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
#include <api/storage/IMapManager.h>

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
    auto keyFramesMgr = cmpMgr->resolve<SolAR::api::storage::IKeyframesManager>();
    auto otherKeyFramesMgr = cmpMgr->resolve<SolAR::api::storage::IKeyframesManager>();
	
    if (keyFramesMgr) {
        std::cout<<"IKeyframesManager instance"<<std::endl;
    }

	auto pointCloudMgr = cmpMgr->resolve<SolAR::api::storage::IPointCloudManager>();
	auto otherPointCloudMgr = cmpMgr->resolve<SolAR::api::storage::IPointCloudManager>();
    if (pointCloudMgr) {
        std::cout<<"IPointCloudManager instance"<<std::endl;
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
    std::cout<<"memory mapping :" << std::endl;
    std::cout<<"--> IPointCloudManager first instance address = "<<&*pointCloudMgr<<std::endl;
    std::cout<<"--> IPointCloudManager second instance address = "<<&*otherPointCloudMgr<<std::endl;
    std::cout<<"--> IKeyframesManager first instance address = "<<&*keyFramesMgr<<std::endl;
    std::cout<<"--> IKeyframesManager second instance address = "<<&*otherKeyFramesMgr<<std::endl;

    return 0;
}
