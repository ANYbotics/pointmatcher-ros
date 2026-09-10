#pragma once

namespace pointmatcher_ros
{

/**
 * @brief Register the ANYbotics data points filters of this package in the libpointmatcher registrar, so that they can be
 *        selected by name in filter chain yaml files like the built in filters. Idempotent and thread safe; called by
 *        PointMatcherFilterInterface before any chain is read, and available to other users of the registrar.
 */
void registerAnyboticsDataPointsFilters();

}  // namespace pointmatcher_ros
