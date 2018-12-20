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
namespace MODULES {
namespace TOOLS {

SolAR3DTransform::SolAR3DTransform():ComponentBase(xpcf::toUUID<SolAR3DTransform>())
{
    addInterface<api::geom::I3DTransform>(this);
}


SolAR3DTransform::~SolAR3DTransform(){

}

FrameworkReturnCode SolAR3DTransform::transform(const std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation, std::vector<SRef<Point3Df>> & outputPoints) const
{
    Point3Df outputPoint3D;
    Vector4f outputVector4f;

    for (int i = 0;i<inputPoints.size();++i){

        Point3Df inputPoint3D = *(inputPoints.at(i));
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
        outputPoints.push_back(xpcf::utils::make_shared<Point3Df>(outputPoint3D));
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transform(const SRef<PointCloud> inputPointCloud, const Transform3Df transformation, SRef<PointCloud>& outputPointCloud) const
{
    Vector4f outputVector4f;
    std::vector<Point3Df> inputPoints = inputPointCloud->getConstPointCloud();

    if (outputPointCloud == nullptr)
        outputPointCloud = xpcf::utils::make_shared<PointCloud>();
    auto& rawOutputPointCloud = outputPointCloud->getPointCloud();

    rawOutputPointCloud.resize(inputPoints.size());

    for (auto i = 0u; i < inputPoints.size(); i++)
    {
        auto& outputPoint3D = rawOutputPointCloud.at(i);
        Point3Df inputPoint3D = inputPoints.at(i);
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
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(std::vector<SRef<Point3Df>> & inputPoints, const Transform3Df transformation) const
{
    Vector4f outputVector4f;

    for (int i = 0;i<inputPoints.size();++i){

        auto point3D = inputPoints.at(i);
        Vector4f inputVector4f(point3D->getX(),point3D->getY(), point3D->getZ(), 1);
        outputVector4f=transformation*inputVector4f;
        if (outputVector4f[3]!=0) {
            point3D->setX(outputVector4f[0]/outputVector4f[3]);
            point3D->setY(outputVector4f[1]/outputVector4f[3]);
            point3D->setZ(outputVector4f[2]/outputVector4f[3]);
        } else {
            point3D->setX(0);
            point3D->setY(0);
            point3D->setZ(0);
        }
    }
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolAR3DTransform::transformInPlace(SRef<PointCloud> inputPointCloud, const Transform3Df transformation) const
{
    Vector4f outputVector4f;
    auto& inputPoints = inputPointCloud->getPointCloud();

    for (auto i = 0u; i < inputPoints.size(); i++)
    {
        auto& point3D = inputPoints.at(i);
        Vector4f inputVector4f(point3D.getX(),point3D.getY(), point3D.getZ(), 1);
        outputVector4f=transformation*inputVector4f;
        if (outputVector4f[3]!=0) {
            point3D.setX(outputVector4f[0]/outputVector4f[3]);
            point3D.setY(outputVector4f[1]/outputVector4f[3]);
            point3D.setZ(outputVector4f[2]/outputVector4f[3]);
        } else {
            point3D.setX(0);
            point3D.setY(0);
            point3D.setZ(0);
        }
    }
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
