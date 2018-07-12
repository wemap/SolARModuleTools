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

#include "SolARImage2WorldMapper4Marker2D.h"
#include "xpcf/component/ComponentFactory.h"
namespace xpcf  = org::bcom::xpcf;


using namespace org::bcom::xpcf;
XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARImage2WorldMapper4Marker2D);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARImage2WorldMapper4Marker2D::SolARImage2WorldMapper4Marker2D():ComponentBase(xpcf::toUUID<SolARImage2WorldMapper4Marker2D>())
{
    addInterface<api::geom::IImage2WorldMapper>(this);
}


SolARImage2WorldMapper4Marker2D::~SolARImage2WorldMapper4Marker2D(){

}

void SolARImage2WorldMapper4Marker2D::setParameters(const Sizei digitalSize, const Sizef worldSize)
{
    m_digitalSize = digitalSize;
    m_worldSize = worldSize;
}

FrameworkReturnCode SolARImage2WorldMapper4Marker2D::map (const std::vector<SRef<Point2Df>> & digitalPoints, std::vector<SRef<Point3Df>> & worldPoints)
{
    worldPoints.clear();
    Point3Df point_3D;
    if (m_digitalSize.width <= 0 || m_digitalSize.height <= 0)
    {
        LOG_ERROR("SolARImage2WorldMapper::map: marker resolution width or height is equal to zero or negative")
        return FrameworkReturnCode::_ERROR_;
    }
    float width_ratio =  m_worldSize.width / m_digitalSize.width;
    float height_ratio = m_worldSize.height / m_digitalSize.height;
    float half_img_width = (float) m_digitalSize.width / 2;
    float half_img_height = (float) m_digitalSize.height / 2;


    for (int i = 0; i<digitalPoints.size();++i)
    {
        Point2Df point_2D = *(digitalPoints.at(i));
        point_3D.setX((point_2D.getX()-half_img_width)*width_ratio);
        point_3D.setY((point_2D.getY()-half_img_height)*height_ratio);
        point_3D.setZ(0);
        worldPoints.push_back(xpcf::utils::make_shared<Point3Df>(point_3D));
    }
    return FrameworkReturnCode::_SUCCESS;
}

}
}
}
