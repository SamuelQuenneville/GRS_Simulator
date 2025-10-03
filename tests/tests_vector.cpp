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

constexpr float EPS = 1e-6f; // tolerance for float comparisons

TEST(VectorFloatTest, InitializerListConstruction) {
    Vector<float,3> v{1.0f,2.0f,3.0f};
    EXPECT_NEAR(v[0], 1.0f, EPS);
    EXPECT_NEAR(v[1], 2.0f, EPS);
    EXPECT_NEAR(v[2], 3.0f, EPS);

    Vector<float,4> v2{1.0f,2.0f};
    EXPECT_NEAR(v2[0], 1.0f, EPS);
    EXPECT_NEAR(v2[1], 2.0f, EPS);
    EXPECT_NEAR(v2[2], 0.0f, EPS);
    EXPECT_NEAR(v2[3], 0.0f, EPS);
}

TEST(VectorFloatTest, DimensionConstructors) {
    Vector<float,2> v2(1.0f, 2.0f);
    EXPECT_NEAR(v2[0], 1.0f, EPS);
    EXPECT_NEAR(v2[1], 2.0f, EPS);

    Vector<float,3> v3(1.0f, 2.0f, 3.0f);
    EXPECT_NEAR(v3[2], 3.0f, EPS);

    Vector<float,4> v4(1.0f, 2.0f, 3.0f, 4.0f);
    EXPECT_NEAR(v4[3], 4.0f, EPS);
}

TEST(VectorFloatTest, ConversionConstructor) {
    const Vector<int,3> vi{1,2,3};
    Vector<float,3> vf(vi);
    EXPECT_NEAR(vf[0], 1.0f, EPS);
    EXPECT_NEAR(vf[1], 2.0f, EPS);
    EXPECT_NEAR(vf[2], 3.0f, EPS);
}

TEST(VectorFloatTest, ZerosAndSize) {
    auto v = Vector<float,5>::zeros();
    for (size_t i=0; i<v.size(); i++) {
        EXPECT_NEAR(v[i], 0.0f, EPS);
    }
    EXPECT_EQ(v.size(), 5u);
}

TEST(VectorFloatTest, EqualityAndArithmeticOperators) {
    Vector<float,3> v1{1.0f,2.0f,3.0f};
    Vector<float,3> v2{1.0f,2.0f,3.0f};
    Vector<float,3> v3{3.0f,2.0f,1.0f};

    // element-wise comparison instead of operator==
    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 == v3);

    auto sum = v1 + v3;
    Vector<float,3> expectedSum{4.0f,4.0f,4.0f};
    for (size_t i=0; i<3; ++i) EXPECT_NEAR(sum[i], expectedSum[i], EPS);

    auto diff = v1 - v3;
    Vector<float,3> expectedDiff{-2.0f,0.0f,2.0f};
    for (size_t i=0; i<3; ++i) EXPECT_NEAR(diff[i], expectedDiff[i], EPS);

    auto neg = -v1;
    Vector<float,3> expectedNeg{-1.0f,-2.0f,-3.0f};
    for (size_t i=0; i<3; ++i) EXPECT_NEAR(neg[i], expectedNeg[i], EPS);

    v1 += Vector<float,3>{1.0f,1.0f,1.0f};
    Vector<float,3> expectedV1{2.0f,3.0f,4.0f};
    for (size_t i=0; i<3; ++i) EXPECT_NEAR(v1[i], expectedV1[i], EPS);

    v1 -= Vector<float,3>{1.0f,1.0f,1.0f};
    Vector<float,3> expectedV1b{1.0f,2.0f,3.0f};
    for (size_t i=0; i<3; ++i) EXPECT_NEAR(v1[i], expectedV1b[i], EPS);
}

TEST(VectorFloatTest, ScalarMultiplication) {
    const Vector<float,3> v{1.0f,-2.0f,3.0f};
    auto left = 2.0f * v;
    auto right = v * 2.0f;
    Vector<float,3> expected{2.0f,-4.0f,6.0f};
    for (size_t i=0; i<3; ++i) {
        EXPECT_NEAR(left[i], expected[i], EPS);
        EXPECT_NEAR(right[i], expected[i], EPS);
    }
}

TEST(VectorFloatTest, NormAndDot) {
    const Vector<float,3> v{3.0f,4.0f,0.0f};
    EXPECT_NEAR(v.norm(), 5.0f, EPS);

    const Vector<float,3> u{1.0f,2.0f,3.0f};
    EXPECT_NEAR(dot(u,v), 11.0f, EPS);
}

TEST(VectorFloatTest, CrossProduct3D) {
    const Vector<float,3> u{1.0f,0.0f,0.0f};
    const Vector<float,3> v{0.0f,1.0f,0.0f};
    auto crossUV = cross(u,v);
    Vector<float,3> expected{0.0f,0.0f,1.0f};
    for (size_t i=0;i<3;i++) EXPECT_NEAR(crossUV[i], expected[i], EPS);

    auto crossVU = cross(v,u);
    Vector<float,3> expectedNeg{0.0f,0.0f,-1.0f};
    for (size_t i=0;i<3;i++) EXPECT_NEAR(crossVU[i], expectedNeg[i], EPS);
}

TEST(VectorFloatTest, Normalize) {
    const Vector<float,3> v{3.0f,0.0f,0.0f};
    auto n = normalize(v);
    EXPECT_NEAR(n.norm(), 1.0f, EPS);
    EXPECT_NEAR(n[0], 1.0f, EPS);

    Vector<float,3> zero{0.0f,0.0f,0.0f};
    auto n2 = normalize(zero);
    for (size_t i=0;i<3;i++) EXPECT_NEAR(n2[i], zero[i], EPS);
}

TEST(VectorFloatTest, Lerp) {
    const Vector<float,3> a{0.0f,0.0f,0.0f};
    const Vector<float,3> b{10.0f,10.0f,10.0f};
    auto mid = lerp(a,b,0.5f);
    Vector<float,3> expected{5.0f,5.0f,5.0f};
    for (size_t i=0;i<3;i++) EXPECT_NEAR(mid[i], expected[i], EPS);
}

TEST(VectorFloatTest, ToOpenGLRef) {
    const Vector<float,3> v{1.0f,2.0f,3.0f};
    auto ogl = v.toOpenglRef();
    Vector<float,3> expected{1.0f,-3.0f,2.0f};
    for (size_t i=0;i<3;i++) EXPECT_NEAR(ogl[i], expected[i], EPS);
}

TEST(VectorFloatTest, Cast) {
    Vector<int,3> vi{1,2,3};
    auto vf = cast<float,3>(vi);
    for (size_t i=0;i<3;i++) EXPECT_NEAR(vf[i], static_cast<float>(vi[i]), EPS);
}

TEST(VectorFloatTest, StreamOutput) {
    const Vector<float,3> v{1.0f,2.0f,3.0f};
    std::ostringstream oss;
    oss << v;
    EXPECT_EQ(oss.str(), "[1, 2, 3]");
}