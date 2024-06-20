/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <cmath>
#include "gtest/gtest.h"
#include "Mathematics/vector3.h"

TEST(testsVector3, accessElementsBracket) {
    Vector3<double> vec = {1,2,3};

    EXPECT_DOUBLE_EQ(vec[0], 1);
    EXPECT_DOUBLE_EQ(vec[1], 2);
    EXPECT_DOUBLE_EQ(vec[2], 3);
}

TEST(testsVector3, accessElementsGetter) {
    Vector3<double> vec = {1,2,3};

    EXPECT_DOUBLE_EQ(vec.x(), 1);
    EXPECT_DOUBLE_EQ(vec.y(), 2);
    EXPECT_DOUBLE_EQ(vec.z(), 3);
}

TEST(testsVector3, setElementsValues) {
    Vector3<double> vec;

    vec.setX(1);
    vec.setY(2);
    vec.setZ(3);

    EXPECT_DOUBLE_EQ(vec[0], 1);
    EXPECT_DOUBLE_EQ(vec[1], 2);
    EXPECT_DOUBLE_EQ(vec[2], 3);
}

TEST(testsVector3, calculateMagnitude) {
    Vector3<double> vec = {2,3,4};

    EXPECT_DOUBLE_EQ(vec.magnitude(), std::sqrt(29));
}

TEST(testsVector3, normalizePositiveVector) {
    Vector3<double> vec = {2, 3, 4};
    Vector3<double> res = vec.normalize();

    EXPECT_DOUBLE_EQ(res[0], 2/std::sqrt(29));
    EXPECT_DOUBLE_EQ(res[1], 3/std::sqrt(29));
    EXPECT_DOUBLE_EQ(res[2], 4/std::sqrt(29));
}

TEST(testsVector3, normalizeNegativeVector) {
    Vector3<double> vec = {-2, -3, -4};
    Vector3<double> res = vec.normalize();

    EXPECT_DOUBLE_EQ(res[0], -2/std::sqrt(29));
    EXPECT_DOUBLE_EQ(res[1], -3/std::sqrt(29));
    EXPECT_DOUBLE_EQ(res[2], -4/std::sqrt(29));
}

TEST(testsVector3, dotProductPositiveVector) {
    Vector3<double> vec1 = {2, 3, 4};
    Vector3<double> vec2 = {5, 6, 7};

    EXPECT_DOUBLE_EQ(vec1.dot(vec2), 56);
}

TEST(testsVector3, dotProductNegativeVector) {
    Vector3<double> vec1 = {-2, -3, -4};
    Vector3<double> vec2 = {-5, -6, -7};

    EXPECT_DOUBLE_EQ(vec1.dot(vec2), 56);
}

TEST(testsVector3, additionResultNewVector) {
    Vector3<double> vec1 = {1, 2, 3};
    Vector3<double> vec2 = {1, 2, 3};

    Vector3<double> res = vec1 + vec2;

    EXPECT_DOUBLE_EQ(res[0], 2);
    EXPECT_DOUBLE_EQ(res[1], 4);
    EXPECT_DOUBLE_EQ(res[2], 6);
}

TEST(testsVector3, additionToItself) {
    Vector3<double> vec1 = {0, 0, 0};
    Vector3<double> vec2 = {1, 2, 3};

    vec1 += vec2;

    EXPECT_DOUBLE_EQ(vec1[0], 1);
    EXPECT_DOUBLE_EQ(vec1[1], 2);
    EXPECT_DOUBLE_EQ(vec1[2], 3);
}

TEST(testsVector3, substractionWithVector) {
    Vector3<double> vec1 = {1, 2, 3};
    Vector3<double> vec2 = {1, 2, 3};

    Vector3<double> res = vec1 - vec2;

    EXPECT_DOUBLE_EQ(res[0], 0);
    EXPECT_DOUBLE_EQ(res[1], 0);
    EXPECT_DOUBLE_EQ(res[2], 0);
}

TEST(testsVector3, multiplicationWithScalar) {
    Vector3<double> vec1 = {1, 2, 3};
    double scalar = 5;

    Vector3<double> res1 = scalar * vec1;
    EXPECT_DOUBLE_EQ(res1[0], 5);
    EXPECT_DOUBLE_EQ(res1[1], 10);
    EXPECT_DOUBLE_EQ(res1[2], 15);

    Vector3<double> res2 = vec1 * scalar;
    EXPECT_DOUBLE_EQ(res2[0], 5);
    EXPECT_DOUBLE_EQ(res2[1], 10);
    EXPECT_DOUBLE_EQ(res2[2], 15);
}

TEST(testsVector3, crossProductPositveValue) {
    Vector3<double> vec1 = {1, 2, 3};
    Vector3<double> vec2 = {4, 5, 6};

    Vector3<double> res = vec1.cross(vec2);

    EXPECT_DOUBLE_EQ(res[0], -3);
    EXPECT_DOUBLE_EQ(res[1],  6);
    EXPECT_DOUBLE_EQ(res[2], -3);
}

TEST(testsVector3, crossProductNegativeValue) {
    Vector3<double> vec1 = {-1, -2, -3};
    Vector3<double> vec2 = {-4, -5, -6};

    Vector3<double> res = vec1.cross(vec2);

    EXPECT_DOUBLE_EQ(res[0], -3);
    EXPECT_DOUBLE_EQ(res[1],  6);
    EXPECT_DOUBLE_EQ(res[2], -3);
}