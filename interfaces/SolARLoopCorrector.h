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

#ifndef SOLARLOOPCORRECTOR_H
#define SOLARLOOPCORRECTOR_H

#include "api/loop/ILoopCorrector.h"


#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"
#include <fstream>

#include <mutex>


namespace SolAR {
// using namespace datastructure;
namespace MODULES {
namespace TOOLS {


/**
 * @class SolARLoopCorrector
 * @brief TODO
 */
class SOLAR_TOOLS_EXPORT_API SolARLoopCorrector : public org::bcom::xpcf::ComponentBase,
        public api::loop::ILoopCorrector {
public:

    SolARLoopCorrector();
    ~SolARLoopCorrector() = default;

    
	void unloadComponent () override final;

 private:
   

};

}
}
}

#endif // SOLARLOOPCORRECTOR_H
