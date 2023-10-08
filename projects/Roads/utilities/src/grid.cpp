#include "roads/grid.h"
#include "Eigen/Eigen"
#include "boost/graph/floyd_warshall_shortest.hpp"

#include "roads/thirdparty/tinysplinecxx.h"
#include <random>

using namespace roads;

double roads::EuclideanDistance(const Point &Point1, const Point &Point2) {
    return std::sqrt(std::pow(Point2.x() - Point1.x(), 2) + std::pow(Point2.y() - Point1.y(), 2) + std::pow(Point2.z() - Point1.z(), 2));
}

double roads::EuclideanDistance(const Point2D &Point1, const Point2D &Point2) {
    return std::sqrt(std::pow(Point2.x() - Point1.x(), 2) + std::pow(Point2.y() - Point1.y(), 2));
}

Eigen::Vector3f ToEigen(const tinyspline::Vec3& v3) {
    return Eigen::Vector3f(v3.x(), v3.y(), v3.z());
}

DynamicGrid<SlopePoint> roads::CalculateSlope(const DynamicGrid<HeightPoint> &InHeightField) {
    constexpr static std::array<int32_t, 4> XDirection4 = {-1, 0, 1, 0};
    constexpr static std::array<int32_t, 4> YDirection4 = {0, 1, 0, -1};
    constexpr static size_t DirectionSize = std::max(XDirection4.size(), YDirection4.size());

    const size_t SizeX = InHeightField.Nx;
    const size_t SizeY = InHeightField.Ny;

    DynamicGrid<SlopePoint> Result(SizeX, SizeY);

#pragma omp parallel for
    for (int32_t y = 0; y < SizeY; ++y) {
        for (int32_t x = 0; x < SizeX; ++x) {
            const size_t OriginIdx = x + y * SizeX;
            double MaxSlope = 0.0;
            for (int32_t Direction = 0; Direction < DirectionSize; ++Direction) {
                const int32_t nx = x + XDirection4[Direction];
                const int32_t ny = y + YDirection4[Direction];
                if (nx < 0 || ny < 0) continue;

                const size_t idx = nx + ny * SizeX;
                if (idx >= InHeightField.size()) continue;

                const double HeightDiff = InHeightField[OriginIdx] - InHeightField[idx];
                const double Distance = std::sqrt(1 + HeightDiff * HeightDiff);
                MaxSlope = std::max(MaxSlope, HeightDiff / Distance);
            }
            Result[OriginIdx] = MaxSlope;
        }
    }

    return Result;
}

tinyspline::BSpline spline::GenerateBSplineFromSegment(const ArrayList<std::array<float, 3>> &InPoints, const ArrayList<std::array<int, 2>> &Segments) {
    using namespace tinyspline;

    ArrayList<float> Points;

    for (const auto &Seg: Segments) {
        Points.insert(std::end(Points), {float(InPoints[Seg[0]][0]), float(InPoints[Seg[0]][1]), float(InPoints[Seg[0]][2])});
    }
    Points.insert(std::end(Points), {float(InPoints[Segments[Segments.size() - 1][1]][0]), float(InPoints[Segments[Segments.size() - 1][1]][1]), float(InPoints[Segments[Segments.size() - 1][1]][2])});

    return BSpline::interpolateCatmullRom(Points, 3);
}

ArrayList<Eigen::Vector3f> spline::GenerateAndSamplePointsFromSegments(const class tinyspline::BSpline& Spline, int32_t SamplePoints) {
    float Step = 1.0f / float(SamplePoints);

    ArrayList<Eigen::Vector3f> Result;
    Result.resize(SamplePoints);

#pragma omp parallel for
    for (int32_t i = 0; i < SamplePoints; i++) {
        auto Point = Spline.eval(float(i) * Step).resultVec3();
        Result[i] = Eigen::Vector3f{Point.x(), Point.y(), Point.z()};
    }

    return Result;
}

float spline::Distance(const Eigen::Vector3d &point, const tinyspline::BSpline &bSpline, float t) {
    tinyspline::DeBoorNet net = bSpline.eval(t);
    auto Result = net.resultVec3();

    Eigen::Vector3d splinePoint(Result.x(), Result.y(), Result.z());
    return float((splinePoint - point).norm());
}

float spline::FindNearestPoint(const Eigen::Vector3d &point, const tinyspline::BSpline &bSpline, float& t, float step, float tolerance) {
    // initial guess
    t = 0.0;

    while (step > tolerance) {
        float f_current = Distance(point, bSpline, t);
        float nextT = t + step;
        if (nextT > 1) {
            nextT = 1;
        }
        float f_next = Distance(point, bSpline, nextT);

        if (f_next < f_current) {
            t = nextT;
        } else {
            step *= 0.5;
        }
    }

    return Distance(point, bSpline, t);
}

ArrayList<float> spline::CalcRoadMask(const std::vector<std::array<float, 3>> &Points, const tinyspline::BSpline &SplineQwQ, float MaxDistance) {
    ArrayList<float> Result;
    Result.resize(Points.size(), std::numeric_limits<float>::max() - 1);

#pragma omp parallel for
    for (int32_t i = 0; i < Points.size(); ++i) {
        float t = 0;
        const std::array<float, 3>& zp = Points[i];
        Eigen::Vector3d ep(zp[0], zp[1], zp[2]);
        //float Distance = spline::FindNearestPoint(ep, SplineQwQ, t);
        float Distance = spline::FindNearestPointSA(ep, SplineQwQ);

        if (std::abs<float>(Distance) < MaxDistance) {
            Result[i] = t;
        }
    }

    return Result;
}

float spline::FindNearestPointSA(const Eigen::Vector3d &Point, const tinyspline::BSpline &Spline) {
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<float> distr(0.0, 1.0);

    float T = 1.0; // initial temperature
    float T_min = 0.001; // minial temperature
    float CoolingRate = 0.99; // sink rate

    float t = distr(gen); // select a initial value
    float BestT = t; // best value

    while (T > T_min) {
        float NewT = distr(gen); // generate new value
        float CurrentDistance = Distance(Point, Spline, t); // distance to current value
        float NewDistance = Distance(Point, Spline, NewT); // distance to new value
        float dE = NewDistance - CurrentDistance; // delta value

        // Randomly accept new value even if become worse
        if (dE < 0 || distr(gen) < std::exp(-dE / T)) {
            t = NewT;
            if (NewDistance < Distance(Point, Spline, BestT)) {
                BestT = NewT; // update best value
            }
        }
        T *= CoolingRate; // decrease temperature
    }

    return Distance(Point, Spline, BestT);
}

#define TINY2EIGEN_VECTOR3F(Name, Expr) auto Name##_ = Expr .resultVec3(); Eigen::Vector3f Name { Name##_.x(), Name##_.y(), Name##_.z() };

Eigen::Vector3f CalculateTangent(Eigen::Vector3f& currentPosition, Eigen::Vector3f& nextPosition, Eigen::Vector3f& prevPosition, const tinyspline::BSpline& Spline, float& t, size_t& i, size_t& SampleCount){
    TINY2EIGEN_VECTOR3F(PrevPosition, Spline.eval(std::clamp(t - 1.0f / static_cast<float>(SampleCount - 1), 0.f, 1.f)));
    TINY2EIGEN_VECTOR3F(NextPosition, Spline.eval(std::clamp(t + 1.0f / static_cast<float>(SampleCount - 1), 0.f, 1.f)));
    Eigen::Vector3f tangent = (NextPosition - PrevPosition).normalized();
    return tangent;
}

void spline::BuildRoadMesh(
    size_t SampleCount,
    float RoadWidth,
    float RoadHeight,
    const tinyspline::BSpline& Spline,
    std::vector<std::array<float, 3>> &OutVertices,
    std::vector<std::array<int, 3>> &OutTriangles
){
    // Init previous tangent as direction to second control point
    Eigen::Vector3f prevTangent;
    TINY2EIGEN_VECTOR3F(p0, Spline.eval(0.0));
    TINY2EIGEN_VECTOR3F(p1, Spline.eval(1.0/(SampleCount-1)));
    prevTangent = (p1 - p0).normalized();

    for (size_t i = 0; i < SampleCount; ++i) {
        float t = static_cast<float>(i) / (SampleCount - 1);
        TINY2EIGEN_VECTOR3F(position, Spline.eval(t));

        // Step forward to estimate next tangent
        TINY2EIGEN_VECTOR3F(p_next, Spline.eval(std::clamp(t + 1.0/(SampleCount-1), 0.0, 1.0)));
        Eigen::Vector3f nextTangent = (p_next - position).normalized();

        // The actual tangent is average of previous and next
        Eigen::Vector3f tangent = (prevTangent + nextTangent).normalized();
        prevTangent = tangent;

        // Calculate normal and binormal
        Eigen::Vector3f normal = Eigen::Vector3f::UnitY() - (Eigen::Vector3f::UnitY().dot(tangent))*tangent;
        normal.normalize();

        Eigen::Vector3f binormal = tangent.cross(normal);

        // left (-) and right (+) vertices
        Eigen::Vector3f roadLeft = position - binormal * RoadWidth * 0.5;
        Eigen::Vector3f roadRight = position + binormal * RoadWidth * 0.5;

        OutVertices.push_back({roadLeft[0], roadLeft[1], roadLeft[2]});
        OutVertices.push_back({roadRight[0], roadRight[1], roadRight[2]});

        // Connect vertices to quads unless it's the first pair
        if (i != 0) {
            int a = (i - 1) * 2;       // Index of first vertex of previous pair (left)
            int b = a + 1;             // Index of second vertex of previous pair (right)
            int c = i * 2;             // Index of first vertex of current pair (left)
            int d = c + 1;             // Index of second vertex of current pair (right)

            // Quad as two triangles: (a,b,c) and (c,b,d)
            OutTriangles.push_back({a, b, c});
            OutTriangles.push_back({c, b, d});
        }
    }
}

std::vector<tinyspline::real> GenerateSamples(int32_t SamplePoints) {
    std::vector<tinyspline::real> res;
    res.resize(SamplePoints);

    for (int32_t i = 0; i < SamplePoints; ++i) {
        res[i] = float(i) / float(SamplePoints);
    }

    return res;
}

ArrayList<spline::FrenetFrame> spline::SampleFrenetFrame(const tinyspline::BSpline &Spline, int32_t SamplePoints) {
    ArrayList<spline::FrenetFrame> Result;

    //float Step = 1.0f / float(SamplePoints);
    std::vector<tinyspline::real> knots = Spline.equidistantKnotSeq(SamplePoints);
    Result.resize(knots.size());

    //tinyspline::BSpline derived = Spline.derive(1, -1e-6f);
    //tinyspline::BSpline derived2 = Spline.derive(2, -1e-6f);

    //for (int32_t i = 0; i < SamplePoints; i++) {
    for (int32_t i = 0; i < knots.size(); ++i) {
        float t = knots[i];

        Eigen::Vector3f position = ToEigen(Spline.eval(t).resultVec3());
        Eigen::Vector3f tangent = CalcTangent(Spline, t, 1.0f / float(SamplePoints) * 1e-2f);
        Eigen::Vector3f normal = CalcNormal(Spline, t, 1.0f / float(SamplePoints) * 1e-2f);
        //Eigen::Vector3f normal {0.f, 1.f, 0.f};

        Result[i] = spline::FrenetFrame {
            position,
            tangent,
            normal,
            tangent.cross(normal).normalized(),
        };
    }

    for (int32_t i = 1; i < Result.size() - 1; ++i) {
        Result[i].Tangent = ((Result[i - 1].Tangent + Result[i].Tangent + Result[i + 1].Tangent) * 0.33333).normalized();
        Result[i].Normal = ((Result[i - 1].Normal + Result[i].Normal + Result[i + 1].Normal) * 0.33333).normalized();
        Result[i].Binormal = Result[i].Tangent.cross(Result[i].Normal).normalized();
    }

    return Result;
}

tinyspline::BSpline spline::Tension(const tinyspline::BSpline &Spline, float Beta) {
    return Spline.tension(Beta);
}

Eigen::Vector3f spline::CalcTangent(const tinyspline::BSpline &Spline, float t, float delta) {
    Eigen::Vector3f vec1 = ToEigen(Spline.eval(std::max(0.f, t - delta)).resultVec3());
    Eigen::Vector3f vec2 = ToEigen(Spline.eval(std::min(1.f, t + delta)).resultVec3());

    Eigen::Vector3f tangent = SAFE_DIV((vec2 - vec1), 2 * delta);
    return tangent.normalized();
}

Eigen::Vector3f spline::CalcNormal(const tinyspline::BSpline &Spline, float t, float delta) {
    Eigen::Vector3f tangent1 = CalcTangent(Spline, t);
    Eigen::Vector3f tangent2 = CalcTangent(Spline, std::min(1.f, t + delta));

    Eigen::Vector3f curvature = (tangent2 - tangent1) / delta;

    Eigen::Vector3f normal = tangent1.cross(curvature);

    return normal.normalized();
}

Eigen::Vector3f spline::CalcPosition(const tinyspline::BSpline &Spline, float t) {
    return ToEigen(Spline.eval(t).resultVec3());
}

class tinyspline::BSpline spline::SubSpline(const tinyspline::BSpline &Spline, float a, float b) {
    return Spline.subSpline(a, b);
}

size_t spline::NumControlPoints(const tinyspline::BSpline &Spline) {
    return Spline.numControlPoints() / Spline.dimension();
}

std::array<float, 3> spline::ControlPoint3At(const tinyspline::BSpline &Spline, size_t index) {
    auto s = Spline.controlPointVec3At(index);
    return { s.x(), s.y(), s.z() };
}
