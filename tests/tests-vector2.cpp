/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include <cmath>
#include "gtest/gtest.h"
#include "Mathematics/vector2.h"

TEST(testsVector2, accessElementsBracket) {
    Vector2<double> vec = {1,2};

    EXPECT_DOUBLE_EQ(vec[0], 1);
    EXPECT_DOUBLE_EQ(vec[1], 2);
}

TEST(testsVector2, accessElementsGetter) {
    Vector2<double> vec = {1,2};

    EXPECT_DOUBLE_EQ(vec.x(), 1);
    EXPECT_DOUBLE_EQ(vec.y(), 2);
}

TEST(testsVector2, setElementsValues) {
    Vector2<double> vec;

    vec.setX(1);
    vec.setY(2);

    EXPECT_DOUBLE_EQ(vec[0], 1);
    EXPECT_DOUBLE_EQ(vec[1], 2);
}

TEST(testsVector2, calculateMagnitude) {
    Vector2<double> vec = {2,3};

    EXPECT_DOUBLE_EQ(vec.length(), std::sqrt(13));
}

TEST(testsVector2, normalizePositiveVector) {
    Vector2<double> vec = {2,3};
    Vector2<double> res = vec.normalize();

    EXPECT_DOUBLE_EQ(res[0], 2/std::sqrt(13));
    EXPECT_DOUBLE_EQ(res[1], 3/std::sqrt(13));
}

TEST(testsVector2, normalizeNegativeVector) {
    Vector2<double> vec = {-2,-3};
    Vector2<double> res = vec.normalize();

    EXPECT_DOUBLE_EQ(res[0], -2/std::sqrt(13));
    EXPECT_DOUBLE_EQ(res[1], -3/std::sqrt(13));
}

TEST(testsVector2, dotProductPositveVector) {
    Vector2<double> vec1 = {2,3};
    Vector2<double> vec2 = {4,5};

    EXPECT_DOUBLE_EQ(vec1.dot(vec2), 23);
}

TEST(testsVector2, dotProductNegativeVector) {
    Vector2<double> vec1 = {-2,-3};
    Vector2<double> vec2 = {-4,-5};

    EXPECT_DOUBLE_EQ(vec1.dot(vec2), 23);
}

TEST(testsVector2, additionResultNewVector) {
    Vector2<double> vec1 = {1,2};
    Vector2<double> vec2 = {1,2};

    Vector2<double> res = vec1 + vec2;

    EXPECT_DOUBLE_EQ(res[0], 2);
    EXPECT_DOUBLE_EQ(res[1], 4);
}

TEST(testsVector2, additionToItself) {
    Vector2<double> vec1 = {0,0};
    Vector2<double> vec2 = {1,2};

    vec1 += vec2;

    EXPECT_DOUBLE_EQ(vec1[0], 1);
    EXPECT_DOUBLE_EQ(vec1[1], 2);
}

TEST(testsVector2, substractionWithVector) {
    Vector2<double> vec1 = {1,2};
    Vector2<double> vec2 = {1,2};

    Vector2<double> res = vec1 - vec2;

    EXPECT_DOUBLE_EQ(res[0], 0);
    EXPECT_DOUBLE_EQ(res[1], 0);
}

TEST(testsVector2, multiplicationWithScalar) {
    Vector2<double> vec1 = {1,1};
    double scalar = 5;

    Vector2<double> res1 = scalar * vec1;
    EXPECT_DOUBLE_EQ(res1[0], 5);
    EXPECT_DOUBLE_EQ(res1[1], 5);

    Vector2<double> res2 = vec1 * scalar;
    EXPECT_DOUBLE_EQ(res2[0], 5);
    EXPECT_DOUBLE_EQ(res2[1], 5);
}