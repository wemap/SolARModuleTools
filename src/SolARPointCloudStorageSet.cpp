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

#include "SolARPointCloudStorageSet.h"
#include "xpcf/component/ComponentFactory.h"

namespace xpcf  = org::bcom::xpcf;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::SolARPointCloudStorageSet);

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

SolARPointCloudStorageSet::SolARPointCloudStorageSet():ComponentBase(xpcf::toUUID<SolARPointCloudStorageSet>())
{
   addInterface<api::storage::IPointCloudStorage>(this);
}


FrameworkReturnCode SolARPointCloudStorageSet::AddPoint(const SRef<Point3Df> point)
{
    m_points.insert(point);
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudStorageSet::SuppressPoint(const SRef<Point3Df> point)
{
    m_points.erase(point);
    return FrameworkReturnCode::_SUCCESS;
}

FrameworkReturnCode SolARPointCloudStorageSet::GetAllPoints(std::vector<SRef<Point3Df>>& points)
{
    points.clear();
    points.assign(m_points.begin(), m_points.end());
    return FrameworkReturnCode::_SUCCESS;
}

bool SolARPointCloudStorageSet::ExistPoint(SRef<Point3Df> point)
{
    return (m_points.find(point) != m_points.end());
}


int SolARPointCloudStorageSet::GetNbPoints()
{
    return m_points.size();
}

}
}
}
