#include "KeyFrameRetrieverMock.h"
#include <core/Log.h>
#include<boost/filesystem.hpp>

namespace xpcf = org::bcom::xpcf;
namespace fs = boost::filesystem;

XPCF_DEFINE_FACTORY_CREATE_INSTANCE(SolAR::MODULES::TOOLS::KeyFrameRetrieverMock)

namespace SolAR {
using namespace datastructure;
namespace MODULES {
namespace TOOLS {

KeyFrameRetrieverMock::KeyFrameRetrieverMock():ConfigurableBase(xpcf::toUUID<KeyFrameRetrieverMock>())
{
    declareInterface<api::reloc::IKeyframeRetriever>(this);

    declareProperty("VOCpath",m_VOCPath);
    declareProperty("threshold", m_threshold);
    declareProperty("level", m_level);
    declareProperty("matchingDistanceRatio", m_distanceRatio);
    declareProperty("matchingDistanceMAX", m_distanceMax);

   LOG_DEBUG("KeyFrameRetrieverMock constructor");
}

KeyFrameRetrieverMock::~KeyFrameRetrieverMock()
{
    LOG_DEBUG(" KeyFrameRetrieverMock destructor")
}

xpcf::XPCFErrorCode KeyFrameRetrieverMock::onConfigured()
{
    LOG_DEBUG(" KeyFrameRetrieverMock onConfigured");

    // Load a vocabulary from m_VOCpath
    if (!fs::exists(m_VOCPath)) {
        LOG_DEBUG(" KeyFrameRetrieverMock onConfigured: the vocabulary file doesn't exists ");
        return xpcf::_ERROR_INVALID_ARGUMENT;
    }

    return xpcf::_SUCCESS;
}

FrameworkReturnCode KeyFrameRetrieverMock::addKeyframe(const SRef<Keyframe>& keyframe)
{
    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode KeyFrameRetrieverMock::suppressKeyframe(uint32_t keyframe_id)
{


    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode KeyFrameRetrieverMock::retrieve(const SRef<Frame>& frame, std::vector<uint32_t> &retKeyframes_id)
{


    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode KeyFrameRetrieverMock::retrieve(const SRef<Frame>& frame, std::set<unsigned int> &canKeyframes_id, std::vector<uint32_t> & retKeyframes_id)
{

    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode KeyFrameRetrieverMock::saveToFile(const std::string& file)
{
    LOG_WARNING("Coming soon!");
    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode KeyFrameRetrieverMock::loadFromFile(const std::string& file)
{
    LOG_WARNING("Coming soon!");
    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}

FrameworkReturnCode KeyFrameRetrieverMock::match(const SRef<Frame>& frame, const SRef<Keyframe>& keyframe, std::vector<DescriptorMatch> &matches)
{

    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}
FrameworkReturnCode KeyFrameRetrieverMock::match(const std::vector<int> &indexDescriptors, const SRef<DescriptorBuffer> &descriptors, const SRef<Keyframe> &keyframe, std::vector<DescriptorMatch> &matches)
{

    return FrameworkReturnCode::_NOT_IMPLEMENTED;
}


}
}
}
