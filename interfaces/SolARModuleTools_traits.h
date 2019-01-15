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

#ifndef SOLARMODULETOOLS_TRAITS_H
#define SOLARMODULETOOLS_TRAITS_H

#include "xpcf/component/ComponentTraits.h"

namespace SolAR {
namespace MODULES {
namespace TOOLS {
class SolARHomographyValidation;
class SolARImage2WorldMapper4Marker2D;
class SolARSBPatternReIndexer;
class SolARKeypointsReIndexer;
class SolARKeyframeSelector;
class SolAR2DTransform;
class SolAR3DTransform;
class SolARBasicMatchesFilter;
class SolARMapFilter;
class SolARMapper;
class SolARBasicSink;
class ThirdPartyConnector;
}
}
}

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARHomographyValidation,
                             "112f9f03-79c1-4393-b8f3-e02227bebfed",
                             "SolARHomographyValidation",
                             "SolAR::MODULES::TOOLS::SolARHomographyValidation component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D,
                             "6fed0169-4f01-4545-842a-3e2425bee248",
                             "SolARImage2WorldMapper4Marker2D",
                             "SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARSBPatternReIndexer,
                             "a2ef5542-029e-4fce-9974-0aea14b29d6f",
                             "SolARSBPatternReIndexer",
                             "SolAR::MODULES::TOOLS::SolARSBPatternReIndexer component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARKeyframeSelector,
                             "ad59a5ba-beb8-11e8-a355-529269fb1459",
                             "SolARKeyframeSelector",
                             "SolAR::MODULES::TOOLS::SolARKeyframeSelector component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARKeypointsReIndexer,
                             "c2836cc0-0344-4956-8959-84936fb4bcf2",
                             "SolARKeypointsReIndexer",
                             "SolAR::MODULES::TOOLS::SolARKeypointsReIndexer component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAR2DTransform,
                             "edcedc0a-9841-4377-aea1-9fa9fdb46fde",
                             "SolAR2DTransform",
                             "SolAR::MODULES::TOOLS::SolAR2DTransform component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolAR3DTransform,
                             "f05dd955-33bd-4d52-8717-93ad298ed3e3",
                             "SolAR3DTransform",
                             "SolAR::MODULES::TOOLS::SolAR3DTransform component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARMapper,
                             "8e3c926a-0861-46f7-80b2-8abb5576692c",
                             "SolARMapper",
                             "SolAR::MODULES::TOOLS::SolARMapper component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARMapFilter,
                             "09205b96-7cba-4415-bc61-64744bc26222",
                             "SolARMapFilter",
                             "SolAR::MODULES::TOOLS::SolARMapFilter component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARBasicMatchesFilter,
                             "cbb620c3-a7fc-42d7-bcbf-f59b475b23b0",
                             "SolARBasicMatchesFilter",
                             "SolAR::MODULES::TOOLS::SolARBasicMatchesFilter component")

XPCF_DEFINE_COMPONENT_TRAITS(SolAR::MODULES::TOOLS::SolARBasicSink,
                             "85db2f25-4f1c-4e06-9011-e020284bfc4f",
                             "SolARBasicSink",
                             "SolAR::MODULES::TOOLS::SolARBasicSink component")

#endif // SOLARMODULETOOLS_TRAITS_H


