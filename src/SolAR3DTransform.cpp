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

#include "SolAR3DTransform.h"
#include "xpcf/component/ComponentFactory.h"
namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolAR3DTransform);

namespace SolAR {
using namespace datastructure;
using namespace api::storage;
using namespace api::solver::map;
namespace MODULES {
namespace TOOLS {

SolAR3DTransform::SolAR3DTransform():ComponentBase(xpcf::toUUID<SolAR3DTransform>())
{
    declareInterface<SolAR::api::geom::I3DTransform>(this);
}


SolAR3DTransform::~SolAR3DTransform(){

}

FrameworkReturnCode SolAR3DTransform::transform(const std::vector<Point3Df> & inputPoints, const Transform3Df & transformation, std::vector<Point3Df> & outputPoints)
{
    Point3Df outputPoint3D;
    Vector4f outputVector4f;

    for (auto inputPoint3D : inputPoints){
        Vector4f inputVector4f(inputPoint3D.getX(),inputPoint3D.getY(), inputPoint3D.getZ(), 1);
        outputVector4f=transformation*inputVector4f;
        if (outputVector4f[3]!=0) {
            outputPoint3D.setX(outputVector4f[0]/outputVector4f[3]);
            outputPoint3D.setY(outputVector4f[1]/outputVector4f[3]);
            outputPoint3D.setZ(outputVector4f[2]/outputVector4f[3]);
        } else {
            outputPoint3D.setX(0);
            outputPoint3D.setY(0);
            outputPoint3D.setZ(0);
        }
        outputPoints.push_back(outputPoint3D);
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const Transform3Df & transformation, std::vector<SRef<CloudPoint>>& pointCloud)
{
    // apply transformation to point cloud
    for (auto &cp : pointCloud) {
        Vector4f inputVector4f(cp->getX(), cp->getY(), cp->getZ(), 1);
        Vector4f outputVector4f = transformation * inputVector4f;
        if (outputVector4f[3] != 0) {
            cp->setX(outputVector4f[0] / outputVector4f[3]);
            cp->setY(outputVector4f[1] / outputVector4f[3]);
            cp->setZ(outputVector4f[2] / outputVector4f[3]);
        }
        else {
            cp->setX(0);
            cp->setY(0);
            cp->setZ(0);
        }
    }

    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(const Transform3Df & transformation, std::vector<SRef<Keyframe>>& keyframes)
{
    // apply transformation to keyframes
    for (auto &kf : keyframes) {
        kf->setPose(transformation * kf->getPose());
    }

    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
