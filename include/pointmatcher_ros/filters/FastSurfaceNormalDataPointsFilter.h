#pragma once

// C++ standard library
#include <string>

// PointMatcher
#include <pointmatcher/PointMatcher.h>

namespace pointmatcher_ros
{

/**
 * @brief Surface normal estimation, drop in replacement of libpointmatcher's SurfaceNormalDataPointsFilter.
 *
 * Same algorithm and same outputs as the reference filter: for every point, the k nearest neighbours
 * (including the point itself) are found, the eigenvector of the smallest eigenvalue of their covariance
 * is the normal, the density is k divided by the volume of the sphere enclosing the centred neighbours,
 * and the local surface descriptors (linearity, planarity, curvature) follow Hackel et al. 2016.
 *
 * Differences are limited to the implementation:
 *  - the nearest neighbour search uses a nanoflann kd-tree built once per cloud instead of libnabo,
 *  - the covariance is accumulated in double and decomposed with a fixed size symmetric eigen solver,
 *    without heap allocations in the per point loop,
 *  - the per point loop is serial: the robot computers are shared with many other processes.
 *
 * Parameters not supported by this implementation (keepEigenValues, keepEigenVectors, keepMeanDist,
 * sortEigen, smoothNormals) are rejected at construction, so a configuration that relies on them fails
 * loudly instead of silently changing behaviour.
 */
template<typename T>
struct FastSurfaceNormalDataPointsFilter : public PointMatcher<T>::DataPointsFilter
{
    using PM = PointMatcher<T>;
    using Parametrizable = PointMatcherSupport::Parametrizable;
    using P = PointMatcherSupport::Parametrizable;
    using Parameters = Parametrizable::Parameters;
    using ParameterDoc = Parametrizable::ParameterDoc;
    using ParametersDoc = Parametrizable::ParametersDoc;
    using InvalidParameter = Parametrizable::InvalidParameter;
    using DataPoints = typename PM::DataPoints;
    using Matrix = typename PM::Matrix;
    using Vector = typename PM::Vector;

    inline static const std::string description()
    {
        return "Surface normal estimation with the same outputs as SurfaceNormalDataPointsFilter (normals, densities, local surface "
               "descriptors), implemented with a nanoflann kd-tree and allocation free per point math.\n\n"
               "Required descriptors: none.\n"
               "Produced descriptors: normals (optional), densities (optional), matchedIds (optional), linearity, planarity, curvature "
               "(optional).\n"
               "Altered descriptors:  none.\n"
               "Altered features:     none.";
    }

    inline static const ParametersDoc availableParameters()
    {
        return { { "knn", "number of nearest neighbors to consider, including the point itself", "5", "3", "2147483647", &P::Comp<unsigned> },
                 { "maxDist", "maximum distance to consider for neighbors", "inf", "0", "inf", &P::Comp<T> },
                 { "epsilon",
                   "approximation to use for the nearest-neighbor search, same meaning as in libnabo (a neighbour may be up to (1 + epsilon) "
                   "times farther than the true one); 0 for an exact search",
                   "0",
                   "0",
                   "inf",
                   &P::Comp<T> },
                 { "keepNormals", "whether the normals should be added as descriptors to the resulting cloud", "1" },
                 { "keepDensities", "whether the point densities should be added as descriptors to the resulting cloud", "0" },
                 { "keepMatchedIds", "whether the identifiers of matches points should be added as descriptors to the resulting cloud", "0" },
                 { "keepSurfaceDescriptors",
                   "whether local surface descriptors (linearity, planarity and curvature from Hackel et al, 2016) should be added to the "
                   "resulting cloud",
                   "1" },
                 { "leafMaxSize", "maximum number of points in a kd-tree leaf", "20", "1", "1024", &P::Comp<unsigned> } };
    }

    const unsigned knn;
    const T maxDist;
    const T epsilon;
    const bool keepNormals;
    const bool keepDensities;
    const bool keepMatchedIds;
    const bool keepSurfaceDescriptors;
    const unsigned leafMaxSize;

    explicit FastSurfaceNormalDataPointsFilter(const Parameters& params = Parameters());
    ~FastSurfaceNormalDataPointsFilter() override = default;

    DataPoints filter(const DataPoints& input) override;
    void inPlaceFilter(DataPoints& cloud) override;
};

}  // namespace pointmatcher_ros
