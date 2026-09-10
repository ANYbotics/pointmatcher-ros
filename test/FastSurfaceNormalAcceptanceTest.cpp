/*
 * FastSurfaceNormalAcceptanceTest.cpp
 *
 * Acceptance test of pointmatcher_ros::FastSurfaceNormalDataPointsFilter against libpointmatcher's
 * SurfaceNormalDataPointsFilter on a recorded lidar cloud (one Velodyne VLP-16 scan of an ANYmal D dataset,
 * test/data/lidar_cloud_200s.pcd, 20208 points, NaN free).
 *
 * Two comparisons:
 *  1. Both filters exact (epsilon 0): the new filter implements the same algorithm, so the neighbour sets must be
 *     identical on every point and normals, densities and surface descriptors equal up to float rounding.
 *  2. Reference in the production configuration (approximate search, epsilon 0.03) against the new filter run exact:
 *     documents the accepted deviation. Where the approximate search of the reference returned a non nearest
 *     neighbour, the outputs differ; everywhere else they agree up to float rounding.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "pointmatcher_ros/filters/FastSurfaceNormalDataPointsFilter.h"
#include "pointmatcher_ros/filters/register_filters.h"
#include "pointmatcher_ros/usings.h"

#ifndef POINTMATCHER_ROS_TEST_DATA_DIR
#error "POINTMATCHER_ROS_TEST_DATA_DIR must be defined by the build system"
#endif

namespace pointmatcher_ros
{

class FastSurfaceNormalAcceptanceTest : public ::testing::Test
{
public:
    using DataPoints = Pm::DataPoints;
    using Label = DataPoints::Label;
    using Labels = DataPoints::Labels;
    using Matrix = Pm::Matrix;

    //! Per point comparison of the two filter outputs.
    struct Comparison
    {
        std::size_t points{ 0 };
        std::size_t identicalNeighbourSets{ 0 };
        std::size_t degenerateReference{ 0 };
        std::size_t degenerateMismatch{ 0 };
        std::vector<double> anglesDeg;  // non degenerate points only
        std::vector<double> anglesDegSameNeighbours;
        double maxDensityRelErrorSameNeighbours{ 0.0 };
        double maxDescriptorAbsErrorSameNeighbours{ 0.0 };
    };

    void SetUp() override
    {
        registerAnyboticsDataPointsFilters();
    }

    //! Minimal reader of the binary PCD written for this test (fields x y z, float32, DATA binary).
    static DataPoints loadRecordedCloud()
    {
        const std::string path{ std::string(POINTMATCHER_ROS_TEST_DATA_DIR) + "/lidar_cloud_200s.pcd" };
        std::ifstream file(path, std::ios::binary);
        if (!file.good())
        {
            throw std::runtime_error("cannot open " + path);
        }
        std::size_t points{ 0 };
        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream stream(line);
            std::string key;
            stream >> key;
            if (key == "POINTS")
            {
                stream >> points;
            }
            else if (key == "FIELDS")
            {
                if (line != "FIELDS x y z")
                {
                    throw std::runtime_error("unexpected PCD fields: " + line);
                }
            }
            else if (key == "DATA")
            {
                std::string format;
                stream >> format;
                if (format != "binary")
                {
                    throw std::runtime_error("unexpected PCD data format: " + format);
                }
                break;
            }
        }
        if (points == 0)
        {
            throw std::runtime_error("PCD header without POINTS");
        }
        std::vector<float> xyz(points * 3u);
        file.read(reinterpret_cast<char*>(xyz.data()), static_cast<std::streamsize>(xyz.size() * sizeof(float)));
        if (!file.good())
        {
            throw std::runtime_error("truncated PCD data");
        }
        Matrix features(4, static_cast<Eigen::Index>(points));
        for (std::size_t i = 0; i < points; ++i)
        {
            features.col(static_cast<Eigen::Index>(i)) << xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2], 1.0f;
        }
        Labels featureLabels;
        featureLabels.push_back(Label("x", 1));
        featureLabels.push_back(Label("y", 1));
        featureLabels.push_back(Label("z", 1));
        featureLabels.push_back(Label("pad", 1));
        return DataPoints(features, featureLabels);
    }

    static Pm::Parameters productionParameters(const std::string& epsilon)
    {
        // pointmatcher_filters_velodyne_vlp16.yaml: knn 10, epsilon 0.03, keepNormals 1, keepDensities 1; the ids and the
        // surface descriptors are added to compare them too.
        Pm::Parameters params;
        params["knn"] = "10";
        params["epsilon"] = epsilon;
        params["keepNormals"] = "1";
        params["keepDensities"] = "1";
        params["keepMatchedIds"] = "1";
        params["keepSurfaceDescriptors"] = "1";
        return params;
    }

    static DataPoints run(const std::string& filterName, const Pm::Parameters& params, const DataPoints& input)
    {
        const auto filter = Pm::get().DataPointsFilterRegistrar.create(filterName, params);
        DataPoints cloud(input);
        filter->inPlaceFilter(cloud);
        return cloud;
    }

    static double percentile(std::vector<double> values, const double p)
    {
        if (values.empty())
        {
            return 0.0;
        }
        std::sort(values.begin(), values.end());
        const auto index = static_cast<std::size_t>(std::min<double>(static_cast<double>(values.size() - 1), std::floor(p * static_cast<double>(values.size()))));
        return values[index];
    }

    static Comparison compare(const DataPoints& reference, const DataPoints& candidate)
    {
        for (const std::string& label : { std::string("normals"), std::string("densities"), std::string("matchedIds"),
                                           std::string("linearity"), std::string("planarity"), std::string("curvature") })
        {
            if (!reference.descriptorExists(label) || !candidate.descriptorExists(label))
            {
                throw std::runtime_error("missing descriptor " + label);
            }
        }
        const auto referenceNormals = reference.getDescriptorViewByName("normals");
        const auto candidateNormals = candidate.getDescriptorViewByName("normals");
        const auto referenceDensities = reference.getDescriptorViewByName("densities");
        const auto candidateDensities = candidate.getDescriptorViewByName("densities");
        const auto referenceIds = reference.getDescriptorViewByName("matchedIds");
        const auto candidateIds = candidate.getDescriptorViewByName("matchedIds");
        const std::vector<std::string> descriptorNames{ "linearity", "planarity", "curvature" };

        Comparison result;
        result.points = static_cast<std::size_t>(reference.features.cols());
        for (Eigen::Index i = 0; i < reference.features.cols(); ++i)
        {
            std::set<float> referenceSet;
            std::set<float> candidateSet;
            for (Eigen::Index j = 0; j < referenceIds.rows(); ++j)
            {
                referenceSet.insert(referenceIds(j, i));
                candidateSet.insert(candidateIds(j, i));
            }
            const bool sameNeighbours{ referenceSet == candidateSet };
            if (sameNeighbours)
            {
                ++result.identicalNeighbourSets;
            }

            const bool referenceDegenerate{ referenceNormals.col(i).norm() < 0.5f };
            const bool candidateDegenerate{ candidateNormals.col(i).norm() < 0.5f };
            if (referenceDegenerate)
            {
                ++result.degenerateReference;
            }
            if (referenceDegenerate != candidateDegenerate)
            {
                ++result.degenerateMismatch;
                continue;
            }
            if (referenceDegenerate)
            {
                continue;
            }
            const double dot{ std::fabs(static_cast<double>(referenceNormals.col(i).dot(candidateNormals.col(i)))) };
            const double angleDeg{ std::acos(std::max(-1.0, std::min(1.0, dot))) * 180.0 / M_PI };
            result.anglesDeg.push_back(angleDeg);
            if (sameNeighbours)
            {
                result.anglesDegSameNeighbours.push_back(angleDeg);
                const double referenceDensity{ static_cast<double>(referenceDensities(0, i)) };
                const double candidateDensity{ static_cast<double>(candidateDensities(0, i)) };
                result.maxDensityRelErrorSameNeighbours =
                    std::max(result.maxDensityRelErrorSameNeighbours,
                             std::fabs(referenceDensity - candidateDensity) / std::max(1e-9, std::fabs(referenceDensity)));
                for (const auto& name : descriptorNames)
                {
                    const double error{ std::fabs(static_cast<double>(reference.getDescriptorViewByName(name)(0, i)) -
                                                  static_cast<double>(candidate.getDescriptorViewByName(name)(0, i))) };
                    result.maxDescriptorAbsErrorSameNeighbours = std::max(result.maxDescriptorAbsErrorSameNeighbours, error);
                }
            }
        }
        return result;
    }

    void record(const Comparison& c)
    {
        RecordProperty("points", static_cast<int>(c.points));
        RecordProperty("identicalNeighbourSets", static_cast<int>(c.identicalNeighbourSets));
        RecordProperty("degenerateReference", static_cast<int>(c.degenerateReference));
        RecordProperty("degenerateMismatch", static_cast<int>(c.degenerateMismatch));
        RecordProperty("angleDegP50", percentile(c.anglesDeg, 0.5));
        RecordProperty("angleDegP99", percentile(c.anglesDeg, 0.99));
        RecordProperty("angleDegMax", c.anglesDeg.empty() ? 0.0 : *std::max_element(c.anglesDeg.begin(), c.anglesDeg.end()));
        RecordProperty("angleDegMaxSameNeighbours",
                       c.anglesDegSameNeighbours.empty() ? 0.0 : *std::max_element(c.anglesDegSameNeighbours.begin(), c.anglesDegSameNeighbours.end()));
        RecordProperty("maxDensityRelErrorSameNeighbours", c.maxDensityRelErrorSameNeighbours);
        RecordProperty("maxDescriptorAbsErrorSameNeighbours", c.maxDescriptorAbsErrorSameNeighbours);
        std::cout << "points " << c.points << ", identical neighbour sets " << c.identicalNeighbourSets << ", degenerate "
                  << c.degenerateReference << " (mismatch " << c.degenerateMismatch << "), angle deg p50 " << percentile(c.anglesDeg, 0.5)
                  << " p99 " << percentile(c.anglesDeg, 0.99) << " max "
                  << (c.anglesDeg.empty() ? 0.0 : *std::max_element(c.anglesDeg.begin(), c.anglesDeg.end())) << ", same neighbours: angle max "
                  << (c.anglesDegSameNeighbours.empty() ? 0.0 : *std::max_element(c.anglesDegSameNeighbours.begin(), c.anglesDegSameNeighbours.end()))
                  << " density rel " << c.maxDensityRelErrorSameNeighbours << " descriptor abs " << c.maxDescriptorAbsErrorSameNeighbours
                  << std::endl;
    }
};

//! Same algorithm: exact reference against the new filter.
TEST_F(FastSurfaceNormalAcceptanceTest, MatchesExactReferenceOnRecordedCloud)
{
    const DataPoints input = loadRecordedCloud();
    ASSERT_EQ(input.features.cols(), 20208);
    const Pm::Parameters params = productionParameters("0");
    const DataPoints reference = run("SurfaceNormalDataPointsFilter", params, input);
    const DataPoints candidate = run("FastSurfaceNormalDataPointsFilter", params, input);
    const Comparison c = compare(reference, candidate);
    record(c);

    EXPECT_EQ(c.identicalNeighbourSets, c.points);
    EXPECT_EQ(c.degenerateMismatch, 0u);
    ASSERT_FALSE(c.anglesDeg.empty());
    // Rounding only: the reference accumulates the covariance in float and uses Eigen's general eigen solver, the new
    // filter accumulates in double and uses the closed form symmetric solver. Observed on this cloud: normal angle max
    // 0.028 deg, density relative error max 2.7e-5, linearity/planarity/curvature absolute error max 1.7e-4 (values in
    // [0, 1]; the ratios of small eigenvalues amplify the rounding of the covariance).
    EXPECT_LT(*std::max_element(c.anglesDeg.begin(), c.anglesDeg.end()), 0.05);
    EXPECT_LT(c.maxDensityRelErrorSameNeighbours, 1e-4);
    EXPECT_LT(c.maxDescriptorAbsErrorSameNeighbours, 1e-3);
}

//! Production configuration: approximate reference (epsilon 0.03) against the new filter run exact. Documents the
//! accepted deviation: outputs differ only where the approximate search returned a non nearest neighbour.
TEST_F(FastSurfaceNormalAcceptanceTest, MatchesProductionReferenceOnRecordedCloudWithinTolerance)
{
    const DataPoints input = loadRecordedCloud();
    const DataPoints reference = run("SurfaceNormalDataPointsFilter", productionParameters("0.03"), input);
    const DataPoints candidate = run("FastSurfaceNormalDataPointsFilter", productionParameters("0"), input);
    const Comparison c = compare(reference, candidate);
    record(c);

    const double identicalFraction{ static_cast<double>(c.identicalNeighbourSets) / static_cast<double>(c.points) };
    EXPECT_GT(identicalFraction, 0.998);
    EXPECT_EQ(c.degenerateMismatch, 0u);
    ASSERT_FALSE(c.anglesDeg.empty());
    EXPECT_LT(percentile(c.anglesDeg, 0.99), 0.1);
    const std::size_t above1Deg{ static_cast<std::size_t>(std::count_if(c.anglesDeg.begin(), c.anglesDeg.end(), [](double a) { return a > 1.0; })) };
    EXPECT_LT(static_cast<double>(above1Deg) / static_cast<double>(c.points), 0.002);
    // Where the neighbour sets agree, the outputs agree up to the rounding documented in the exact test above. Observed on
    // this cloud: 20196 of 20208 neighbour sets identical, normal angle p99 0.02 deg, max 86 deg on the 12 points where
    // the approximate search of the reference had returned a non nearest neighbour.
    EXPECT_LT(*std::max_element(c.anglesDegSameNeighbours.begin(), c.anglesDegSameNeighbours.end()), 0.05);
    EXPECT_LT(c.maxDensityRelErrorSameNeighbours, 1e-4);
    EXPECT_LT(c.maxDescriptorAbsErrorSameNeighbours, 1e-3);
}

}  // namespace pointmatcher_ros
