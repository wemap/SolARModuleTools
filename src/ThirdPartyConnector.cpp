#include <iostream>
#include <utility>

#include "ThirdPartyConnector.h"
#include "SharedCircularBuffer.hpp"

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::ThirdPartyConnector);

namespace xpcf = org::bcom::xpcf;

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

ThirdPartyConnector::ThirdPartyConnector()
    : IThirdPartyConnector(),ComponentBase(xpcf::toUUID<ThirdPartyConnector>())
    , m_buffer( 1 ) //Replace with parameter passed to constructor when supported by XPCF
{
    addInterface<IThirdPartyConnector>(this);
}


ThirdPartyConnector::~ThirdPartyConnector()
{
}

void ThirdPartyConnector::set( const SRef<Transform3Df>& pose, const SRef<Image>& image )
{
    m_buffer.push( std::make_pair(pose, image ) );
}

void ThirdPartyConnector::get( SRef<Transform3Df>& pose, SRef<Image>& image )
{
    connectorDataType output = m_buffer.pop();
    pose = output.first;
    image = output.second;
}

bool ThirdPartyConnector::tryGet( SRef<Transform3Df>& pose, SRef<Image>& image )
{
    connectorDataType output;
    bool popSuccessful = false;
    if( m_buffer.tryPop( output ) )
    {
        pose = output.first;
        image = output.second;
        popSuccessful = true;
    }
    return popSuccessful;
}


}
}
}

