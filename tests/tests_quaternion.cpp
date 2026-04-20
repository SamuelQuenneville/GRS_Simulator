/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "gtest/gtest.h"
#include "Mathematics/math.h"

using namespace grs;

constexpr float EPS = 1e-6f;

TEST(QuaternionFloatTest, ConstructionAndIdentity) {
    constexpr Quaternion<float> q1{1.0f, 2.0f, 3.0f, 4.0f};
    EXPECT_NEAR(q1.x, 1.0f, EPS);
    EXPECT_NEAR(q1.y, 2.0f, EPS);
    EXPECT_NEAR(q1.z, 3.0f, EPS);
    EXPECT_NEAR(q1.w, 4.0f, EPS);

    constexpr auto qId = Quaternion<float>::identity();
    EXPECT_NEAR(qId.x, 0.0f, EPS);
    EXPECT_NEAR(qId.y, 0.0f, EPS);
    EXPECT_NEAR(qId.z, 0.0f, EPS);
    EXPECT_NEAR(qId.w, 1.0f, EPS);
}

TEST(QuaternionFloatTest, Arithmetic) {
    Quaternion<float> q1{1.0f,2.0f,3.0f,4.0f};
    Quaternion<float> q2{0.5f,0.5f,0.5f,0.5f};

    auto sum = q1 + q2;
    EXPECT_NEAR(sum.x, 1.5f, EPS);
    EXPECT_NEAR(sum.y, 2.5f, EPS);
    EXPECT_NEAR(sum.z, 3.5f, EPS);
    EXPECT_NEAR(sum.w, 4.5f, EPS);

    auto diff = q1 - q2;
    EXPECT_NEAR(diff.x, 0.5f, EPS);
    EXPECT_NEAR(diff.y, 1.5f, EPS);
    EXPECT_NEAR(diff.z, 2.5f, EPS);
    EXPECT_NEAR(diff.w, 3.5f, EPS);

    auto scaled = q1 * 2.0f;
    EXPECT_NEAR(scaled.x, 2.0f, EPS);
    EXPECT_NEAR(scaled.y, 4.0f, EPS);
    EXPECT_NEAR(scaled.z, 6.0f, EPS);
    EXPECT_NEAR(scaled.w, 8.0f, EPS);
}

TEST(QuaternionFloatTest, ConjugateAndInverse) {
    Quaternion<float> q{1.0f,2.0f,3.0f,4.0f};
    auto qc = q.conjugate();
    EXPECT_NEAR(qc.x, -1.0f, EPS);
    EXPECT_NEAR(qc.y, -2.0f, EPS);
    EXPECT_NEAR(qc.z, -3.0f, EPS);
    EXPECT_NEAR(qc.w, 4.0f, EPS);

    auto qi = q.inverse();
    Quaternion<float> expected{-0.03333333f, -0.06666667f, -0.1f, 0.13333333f}; // precomputed
    EXPECT_NEAR(qi.x, expected.x, EPS);
    EXPECT_NEAR(qi.y, expected.y, EPS);
    EXPECT_NEAR(qi.z, expected.z, EPS);
    EXPECT_NEAR(qi.w, expected.w, EPS);
}

TEST(QuaternionFloatTest, Normalize) {
    constexpr Quaternion<float> q{1.0f,2.0f,3.0f,4.0f};
    const auto n = normalize(q);
    const float len = std::sqrt(n.x*n.x + n.y*n.y + n.z*n.z + n.w*n.w);
    EXPECT_NEAR(len, 1.0f, EPS);
}

TEST(QuaternionFloatTest, DotProduct) {
    constexpr Quaternion<float> q1{1.0f,0.0f,0.0f,0.0f};
    constexpr Quaternion<float> q2{0.0f,1.0f,0.0f,0.0f};
    EXPECT_NEAR(dot(q1,q2), 0.0f, EPS);
}

TEST(QuaternionFloatTest, EulerConversion) {
    Vector<float,3> euler{0.1f, 0.2f, 0.3f};
    const auto q = eulerToQuat(euler);
    auto eulerBack = quatToEuler(q);

    EXPECT_NEAR(eulerBack[0], euler[0], EPS);
    EXPECT_NEAR(eulerBack[1], euler[1], EPS);
    EXPECT_NEAR(eulerBack[2], euler[2], EPS);
}

TEST(QuaternionFloatTest, VectorRotation) {
    const Quaternion<float> q = eulerToQuat(Vector<float,3>{0.0f, 0.0f, M_PI_2});
    const Vector<float,3> v{1.0f,0.0f,0.0f};
    auto vRot = q * v;
    EXPECT_NEAR(vRot[0], 0.0f, EPS);
    EXPECT_NEAR(vRot[1], 1.0f, EPS);
    EXPECT_NEAR(vRot[2], 0.0f, EPS);
}

TEST(QuaternionFloatTest, SLERP) {
    constexpr Quaternion<float> q1 = Quaternion<float>::identity();
    const Quaternion<float> q2 = eulerToQuat(Vector<float,3>{0.0f, 0.0f, M_PI_2});
    const auto qm = slerp(q1,q2,0.5f);

    const auto v = Vector<float,3>{1.0f,0.0f,0.0f};
    //auto v1 = q1*v;
    auto v2 = qm*v;
    //auto v3 = q2*v;

    // v2 should be approximately halfway between v1 and v3
    EXPECT_NEAR(v2[0], 0.7071067f, EPS);
    EXPECT_NEAR(v2[1], 0.7071067f, EPS);
    EXPECT_NEAR(v2[2], 0.0f, EPS);
}

TEST(QuaternionFloatTest, FrameConversionRoundTrip) {
    constexpr float tol = 1e-5f;

    Vector<float,3> euler{0.1745, 0.2617, 0.0};

    const auto q_ned = eulerToQuat(euler);
    const auto q_yxz = q_ned.nedToYup();
    const auto q_back = q_yxz.yUpToNed();

    auto e_back = quatToEuler(q_back);

    EXPECT_NEAR(e_back[0], euler[0], tol);
    EXPECT_NEAR(e_back[1], euler[1], tol);
    EXPECT_NEAR(e_back[2], euler[2], tol);
}

TEST(QuaternionFloatTest, FrameConversionIdentity) {
    const auto q = Quaternion<float>::identity();

    const auto q2 = q.nedToYup();
    const auto q3 = q2.yUpToNed();

    EXPECT_NEAR(q3.x, q.x, EPS);
    EXPECT_NEAR(q3.y, q.y, EPS);
    EXPECT_NEAR(q3.z, q.z, EPS);
    EXPECT_NEAR(q3.w, q.w, EPS);
}

TEST(QuaternionFloatTest, FrameConversionPureRollConsistency) {
    const float roll = 0.8f;

    const auto q_ned = eulerToQuat(Vector<float,3>{roll, 0.0f, 0.0f});
    const auto q_yxz = q_ned.nedToYup();

    const auto q_back = q_yxz.yUpToNed();
    const auto e = quatToEuler(q_back);

    EXPECT_NEAR(e[0], roll, EPS);
}

TEST(QuaternionFloatTest, FrameConversionVectorConsistency) {
    const Vector<float,3> v_ned{1.0f, 2.0f, -3.0f};

    const auto q = eulerToQuat(Vector<float,3>{0.4f, -0.2f, 0.7f});

    const auto v1 = q * v_ned;

    const auto q_yxz = q.nedToYup();
    const auto v_yxz = q_yxz * v_ned;

    const auto q_back = q_yxz.yUpToNed();
    const auto v_back = q_back * v_ned;

    EXPECT_NEAR(v_back[0], v1[0], 1e-4f);
    EXPECT_NEAR(v_back[1], v1[1], 1e-4f);
    EXPECT_NEAR(v_back[2], v1[2], 1e-4f);
}