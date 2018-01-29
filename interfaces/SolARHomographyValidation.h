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

#ifndef SOLARHOMOGRAPHYVALIDATION_H
#define SOLARHOMOGRAPHYVALIDATION_H

#include "api/solver/pose/IHomographyValidation.h"
#include "ComponentBase.h"
#include "SolARToolsAPI.h"

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

class SOLAR_TOOLS_EXPORT_API SolARHomographyValidation : public org::bcom::xpcf::ComponentBase,
        public api::solver::pose::IHomographyValidation {
public:
    SolARHomographyValidation();
    ~SolARHomographyValidation() = default;

    bool isValid(const std::vector<SRef<Point2Df>>& ref2DSquaredMarkerCorners, const std::vector<SRef<Point2Df>>& projected2DSquaredMarkerCorners) override;
    void unloadComponent () override final;
        XPCF_DECLARE_UUID("112f9f03-79c1-4393-b8f3-e02227bebfed");

};

}
}
}  // end of namespace Solar

#endif // SOLARHOMOGRAPHYVALIDATION_H
