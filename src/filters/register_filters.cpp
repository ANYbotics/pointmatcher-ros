#include "pointmatcher_ros/filters/register_filters.h"

// C++ standard library
#include <mutex>

// PointMatcher
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Registrar.h>

// pointmatcher_ros
#include "pointmatcher_ros/filters/FastSurfaceNormalDataPointsFilter.h"

namespace pointmatcher_ros
{

namespace
{

template<typename T>
void registerFor()
{
    using PM = PointMatcher<T>;
    using Descriptor = typename PointMatcherSupport::Registrar<typename PM::DataPointsFilter>::template GenericClassDescriptor<
        FastSurfaceNormalDataPointsFilter<T>>;
    // libpointmatcher exposes its singleton as const; the registrars are meant to be extended, so the constness is lifted here.
    auto& pm = const_cast<PM&>(PM::get());
    pm.DataPointsFilterRegistrar.reg("FastSurfaceNormalDataPointsFilter", new Descriptor());
}

}  // namespace

void registerAnyboticsDataPointsFilters()
{
    static std::once_flag flag;
    std::call_once(flag, []() {
        registerFor<float>();
        registerFor<double>();
    });
}

}  // namespace pointmatcher_ros
