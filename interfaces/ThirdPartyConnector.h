#ifndef THIRDPARTYCONNECTOR_H
#define THIRDPARTYCONNECTOR_H


#include <string>
#include <utility>

#include "api/sink/IThirdPartyConnector.h"
// Definition of ThirdPartyConnector Class //
// part of SolAR namespace //

#include "xpcf/component/ComponentBase.h"
#include "SharedCircularBuffer.hpp"
#include "SolARToolsAPI.h"


namespace SolAR {
using namespace datastructure;
namespace MODULES {
    namespace TOOLS {

class SOLARFRAMEWORK_API ThirdPartyConnector : public org::bcom::xpcf::ComponentBase,
        public SolAR::api::sink::IThirdPartyConnector {
public:
    ThirdPartyConnector();
    virtual ~ThirdPartyConnector();
    void unloadComponent () override final;


    void set( const SRef<Pose>& pose, const SRef<Image>& image ) override;
    void get( SRef<Pose>& pose, SRef<Image>& image ) override;
    bool tryGet( SRef<Pose>& pose, SRef<Image>& image ) override;

protected:
    typedef std::pair<SRef<Pose>, SRef<Image>> connectorDataType;

    SharedCircularBuffer<connectorDataType> m_buffer;
};

}  // end of namespace SolAR
}
}

#endif // THIRDPARTYCONNECTOR_H
