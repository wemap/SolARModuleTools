#ifndef SolARBasicMatchesFilter_H
#define SolARBasicMatchesFilter_H

#include "api/features/IMatchesFilter.h"
#include "xpcf/component/ComponentBase.h"
#include "SolARToolsAPI.h"

#include <vector>

namespace SolAR {
    namespace MODULES {
        namespace TOOLS {
       /**
       * @class SolARBasicMatchesFilter
       * @brief <B>Retains the best match for each keypoint.</B>
       * <TT>UUID: cbb620c3-a7fc-42d7-bcbf-f59b475b23b0</TT>
       *
       */
            class SOLAR_TOOLS_EXPORT_API SolARBasicMatchesFilter : public org::bcom::xpcf::ComponentBase,
                    public api::features::IMatchesFilter {
            public:
               ///@brief SolARBasicMatchesFilter constructor.
               SolARBasicMatchesFilter();
               ///@brief SolARBasicMatchesFilter destructor.
               ~SolARBasicMatchesFilter() override;

               /// @brief filter matches based on redundancy strategy. This filter removes all the mmulitples matches.
               /// @param[in] Original matches found between two descriptors "desc_1" and "desc_2".
               /// @param[out] Filtred matches based on redanduncy or geometric relations such as epipolar constraint.
               /// @param[in] Original keypoints associated to desc_1.
               /// @param[in] Original keypoints associated to desc_2.
               void filter(const std::vector<datastructure::DescriptorMatch> & inputMatches,
                          std::vector<datastructure::DescriptorMatch> & outputMatches,
                          const std::vector<datastructure::Keypoint> & inputKeyPointsA,
                          const std::vector<datastructure::Keypoint> & inputKeyPointsB) override;

                void unloadComponent () override final;


             private:

            };

}
}
}




#endif // SOLARMATCHESFILTEROPENCV_H
