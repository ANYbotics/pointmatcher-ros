#include "pointmatcher_ros/filters/FastSurfaceNormalDataPointsFilter.h"

// C++ standard library
#include <cmath>
#include <limits>
#include <vector>

// Eigen
#include <Eigen/Eigenvalues>

// nanoflann (header only, fetched at configure time by the nanoflann_vendor package)
#include <nanoflann_vendor/nanoflann.hpp>

namespace pointmatcher_ros
{

namespace
{

//! Contiguous xyz copy of the features for nanoflann (3 values per point instead of the 4 row homogeneous layout of DataPoints),
//! so that a tree traversal touches 25 percent less memory. The copy costs one pass over N points.
template<typename T>
struct FeaturesAdaptor
{
    using Matrix = typename PointMatcher<T>::Matrix;

    explicit FeaturesAdaptor(const Matrix& features) : count_(static_cast<size_t>(features.cols())), xyz_(3 * count_)
    {
        for (size_t i = 0; i < count_; ++i)
        {
            xyz_[3 * i] = features(0, i);
            xyz_[3 * i + 1] = features(1, i);
            xyz_[3 * i + 2] = features(2, i);
        }
    }

    inline size_t kdtree_get_point_count() const { return count_; }
    inline T kdtree_get_pt(const size_t index, const size_t dim) const { return xyz_[3 * index + dim]; }
    template<class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const
    {
        return false;
    }

    size_t count_;
    std::vector<T> xyz_;
};

template<typename T>
using KdTree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<T, FeaturesAdaptor<T>>, FeaturesAdaptor<T>, 3, unsigned>;

}  // namespace

template<typename T>
FastSurfaceNormalDataPointsFilter<T>::FastSurfaceNormalDataPointsFilter(const Parameters& params)
    : PM::DataPointsFilter("FastSurfaceNormalDataPointsFilter", FastSurfaceNormalDataPointsFilter::availableParameters(), params),
      knn(Parametrizable::get<unsigned>("knn")),
      maxDist(Parametrizable::get<T>("maxDist")),
      epsilon(Parametrizable::get<T>("epsilon")),
      keepNormals(Parametrizable::get<bool>("keepNormals")),
      keepDensities(Parametrizable::get<bool>("keepDensities")),
      keepMatchedIds(Parametrizable::get<bool>("keepMatchedIds")),
      keepSurfaceDescriptors(Parametrizable::get<bool>("keepSurfaceDescriptors")),
      leafMaxSize(Parametrizable::get<unsigned>("leafMaxSize"))
{
}

template<typename T>
typename FastSurfaceNormalDataPointsFilter<T>::DataPoints FastSurfaceNormalDataPointsFilter<T>::filter(const DataPoints& input)
{
    DataPoints output(input);
    inPlaceFilter(output);
    return output;
}

template<typename T>
void FastSurfaceNormalDataPointsFilter<T>::inPlaceFilter(DataPoints& cloud)
{
    using Label = typename DataPoints::Label;
    using Labels = typename DataPoints::Labels;
    using View = typename DataPoints::View;

    const Eigen::Index pointsCount{ cloud.features.cols() };
    const Eigen::Index featDim{ cloud.features.rows() };
    if (featDim != 4)
    {
        throw typename DataPoints::InvalidField("FastSurfaceNormalDataPointsFilter: only 3D point clouds (4 feature rows) are supported.");
    }

    // Reserve the output descriptors, in the same order as SurfaceNormalDataPointsFilter does.
    Labels cloudLabels;
    if (keepNormals)
    {
        cloudLabels.push_back(Label("normals", 3));
    }
    if (keepDensities)
    {
        cloudLabels.push_back(Label("densities", 1));
    }
    if (keepMatchedIds)
    {
        cloudLabels.push_back(Label("matchedIds", knn));
    }
    if (keepSurfaceDescriptors)
    {
        cloudLabels.push_back(Label("linearity", 1));
        cloudLabels.push_back(Label("planarity", 1));
        cloudLabels.push_back(Label("curvature", 1));
    }
    cloud.allocateDescriptors(cloudLabels);

    // Views are only valid after the allocation above; boost::optional is what the reference uses, a plain pointer-like scheme is enough here.
    View normals = keepNormals ? cloud.getDescriptorViewByName("normals") : View(cloud.descriptors.block(0, 0, 0, 0));
    View densities = keepDensities ? cloud.getDescriptorViewByName("densities") : View(cloud.descriptors.block(0, 0, 0, 0));
    View matchedIds = keepMatchedIds ? cloud.getDescriptorViewByName("matchedIds") : View(cloud.descriptors.block(0, 0, 0, 0));
    View linearity = keepSurfaceDescriptors ? cloud.getDescriptorViewByName("linearity") : View(cloud.descriptors.block(0, 0, 0, 0));
    View planarity = keepSurfaceDescriptors ? cloud.getDescriptorViewByName("planarity") : View(cloud.descriptors.block(0, 0, 0, 0));
    View curvature = keepSurfaceDescriptors ? cloud.getDescriptorViewByName("curvature") : View(cloud.descriptors.block(0, 0, 0, 0));

    if (pointsCount == 0)
    {
        return;
    }

    // Build the kd-tree once over the 3D part of the features. The adaptor reads the feature matrix in place, no copy.
    const FeaturesAdaptor<T> adaptor(cloud.features);
    KdTree<T> tree(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(leafMaxSize));
    tree.buildIndex();

    // libnabo accepts a neighbour up to (1 + epsilon) times farther than the true one, on distances. nanoflann prunes on squared
    // distances with a factor (1 + eps), so the equivalent factor is (1 + epsilon)^2.
    nanoflann::SearchParameters searchParams;
    searchParams.eps = static_cast<float>((1.0 + static_cast<double>(epsilon)) * (1.0 + static_cast<double>(epsilon)) - 1.0);
    const T maxDistSquared{ std::isinf(maxDist) ? std::numeric_limits<T>::infinity() : maxDist * maxDist };

    // Same degeneracy criterion as the reference in spirit: it requires the covariance to have rank at least 2 (a full pivot QR rank test
    // with Eigen's default threshold, which is proportional to the machine epsilon of T). Here the eigenvalues are available, so the
    // second largest one is compared with the largest one at the same relative precision.
    const double rankThreshold{ 3.0 * static_cast<double>(std::numeric_limits<T>::epsilon()) };

    const auto processRange = [&](const Eigen::Index begin, const Eigen::Index end) {
        std::vector<unsigned> neighbourIds(knn);
        std::vector<T> neighbourDistancesSquared(knn);
        Eigen::Vector3d point;
        Eigen::Vector3d mean;
        Eigen::Matrix3d covariance;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver;

        for (Eigen::Index position = begin; position < end; ++position)
        {
            // Visit the points in the order the kd-tree stores them: consecutive queries then touch the same leaves and the traversal
            // stays in cache. The result of every query is independent of this order.
            const Eigen::Index i{ static_cast<Eigen::Index>(tree.vAcc_[position]) };
            // k nearest neighbours, sorted by increasing distance, the point itself first.
            const T query[3] = { cloud.features(0, i), cloud.features(1, i), cloud.features(2, i) };
            nanoflann::KNNResultSet<T, unsigned> resultSet(knn);
            resultSet.init(neighbourIds.data(), neighbourDistancesSquared.data());
            tree.findNeighbors(resultSet, query, searchParams);
            unsigned realKnn{ static_cast<unsigned>(resultSet.size()) };
            while (realKnn > 0u && neighbourDistancesSquared[realKnn - 1u] > maxDistSquared)
            {
                --realKnn;
            }

            // Mean and covariance of the neighbours (the reference divides by the number of neighbours, not n - 1).
            mean.setZero();
            for (unsigned j = 0; j < realKnn; ++j)
            {
                mean += cloud.features.template block<3, 1>(0, neighbourIds[j]).template cast<double>();
            }
            const double inverseCount{ realKnn > 0u ? 1.0 / static_cast<double>(realKnn) : 0.0 };
            mean *= inverseCount;
            covariance.setZero();
            double maxCentredSquaredNorm{ 0.0 };
            for (unsigned j = 0; j < realKnn; ++j)
            {
                point = cloud.features.template block<3, 1>(0, neighbourIds[j]).template cast<double>() - mean;
                covariance.noalias() += point * point.transpose();
                maxCentredSquaredNorm = std::max(maxCentredSquaredNorm, point.squaredNorm());
            }
            covariance *= inverseCount;

            // Eigen decomposition of the symmetric covariance; eigenvalues in increasing order.
            eigenSolver.computeDirect(covariance);  // closed form, exact to about 1e-8 relative in double
            const Eigen::Vector3d& eigenValues = eigenSolver.eigenvalues();
            const bool isDegenerate{ realKnn < 3u || !(eigenValues(1) > rankThreshold * eigenValues(2)) };

            if (keepNormals)
            {
                if (isDegenerate)
                {
                    normals.col(i).setZero();
                }
                else
                {
                    // Smallest eigenvector, clamped to [-1, 1] like the reference.
                    normals.col(i) = eigenSolver.eigenvectors().col(0).template cast<T>().cwiseMax(T(-1)).cwiseMin(T(1));
                }
            }
            if (keepDensities)
            {
                if (isDegenerate)
                {
                    densities(0, i) = T(0);
                }
                else
                {
                    // Volume of the sphere enclosing the centred neighbours, V = 4/3 pi r^3.
                    const double volume{ (4.0 / 3.0) * M_PI * (maxCentredSquaredNorm * std::sqrt(maxCentredSquaredNorm)) };
                    densities(0, i) = static_cast<T>(volume > 0.0 ? static_cast<double>(realKnn) / volume : 0.0);
                }
            }
            if (keepMatchedIds)
            {
                for (unsigned j = 0; j < knn; ++j)
                {
                    matchedIds(j, i) = j < realKnn ? static_cast<T>(neighbourIds[j]) : static_cast<T>(-1);
                }
            }
            if (keepSurfaceDescriptors)
            {
                if (isDegenerate)
                {
                    linearity(0, i) = T(0);
                    planarity(0, i) = T(0);
                    curvature(0, i) = T(0);
                }
                else
                {
                    const double sigma1{ std::sqrt(std::max(eigenValues(0), 0.0)) };
                    const double sigma2{ std::sqrt(std::max(eigenValues(1), 0.0)) };
                    const double sigma3{ std::sqrt(std::max(eigenValues(2), 0.0)) };
                    linearity(0, i) = static_cast<T>((sigma3 - sigma2) / sigma3);
                    planarity(0, i) = static_cast<T>((sigma2 - sigma1) / sigma3);
                    curvature(0, i) = static_cast<T>(sigma1 / (sigma1 + sigma2 + sigma3));
                }
            }
        }
    };

    processRange(0, pointsCount);
}

template struct FastSurfaceNormalDataPointsFilter<float>;
template struct FastSurfaceNormalDataPointsFilter<double>;

}  // namespace pointmatcher_ros
