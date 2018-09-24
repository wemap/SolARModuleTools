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

#include "SolARBasicMatchesFilter.h"
#include "xpcf/component/ComponentFactory.h"
#include <set>

namespace xpcf = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARBasicMatchesFilter);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARBasicMatchesFilter::SolARBasicMatchesFilter():ComponentBase(xpcf::toUUID<SolARBasicMatchesFilter>())
{
    addInterface<api::features::IMatchesFilter>(this);
}


SolARBasicMatchesFilter::~SolARBasicMatchesFilter(){

}


bool sortMatchByDistance(const std::pair<int,float> &lhs, const std::pair<int,float> &rhs)
{
    return lhs.second < rhs.second;
}

// filter matches : keep the best match in case of multiple matches per keypoint
void SolARBasicMatchesFilter::filter(const std::vector<DescriptorMatch>&inputMatches,
                                            std::vector<DescriptorMatch>&outputMatches,
                                            const std::vector<SRef<Keypoint>>&inputKeyPointsA,
                                            const std::vector<SRef<Keypoint>>&inputKeyPointsB){


    std::vector<DescriptorMatch>matches;
    std::map<int,std::vector<std::pair<int,float>>> matchesMap;

    for(auto itr:inputMatches){
        matchesMap[itr.getIndexInDescriptorA()].push_back(std::make_pair(itr.getIndexInDescriptorB(),itr.getMatchingScore()));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){
            std::sort(ptr.begin(),ptr.end(),sortMatchByDistance);
        }
        matches.push_back(DescriptorMatch(itr->first, ptr.begin()->first,ptr.begin()->second));
    }


    matchesMap.clear();
    for(auto itr:matches){
        matchesMap[itr.getIndexInDescriptorB()].push_back(std::make_pair(itr.getIndexInDescriptorA(),itr.getMatchingScore()));
    }

    matches.clear();
    for (std::map<int,std::vector<std::pair<int,float>>>::iterator itr=matchesMap.begin();itr!=matchesMap.end();++itr){
        std::vector<std::pair<int,float>> ptr=itr->second;
        if(ptr.size()>1){
            std::sort(ptr.begin(),ptr.end(),sortMatchByDistance);
        }
        matches.push_back(DescriptorMatch(ptr.begin()->first,itr->first,ptr.begin()->second));
    }


    outputMatches=matches;


 }

}
}
}
