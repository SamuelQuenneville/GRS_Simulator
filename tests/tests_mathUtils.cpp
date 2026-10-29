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

constexpr double EPS = 1e-12;

// ----------------- degToRad / radToDeg -----------------
TEST(MathUtilsTest, DegToRadBasic) {
    EXPECT_NEAR(degToRad(0.0), 0.0, EPS);
    EXPECT_NEAR(degToRad(180.0), M_PI, EPS);
    EXPECT_NEAR(degToRad(360.0), 2.0 * M_PI, EPS);
    EXPECT_NEAR(degToRad(90.0), M_PI / 2.0, EPS);
}

TEST(MathUtilsTest, RadToDegBasic) {
    EXPECT_NEAR(radToDeg(0.0), 0.0, EPS);
    EXPECT_NEAR(radToDeg(M_PI), 180.0, EPS);
    EXPECT_NEAR(radToDeg(2.0 * M_PI), 360.0, EPS);
    EXPECT_NEAR(radToDeg(M_PI / 2.0), 90.0, EPS);
}

TEST(MathUtilsTest, RoundTripConversions) {
    constexpr double val = 123.456;
    EXPECT_NEAR(radToDeg(degToRad(val)), val, EPS);
    EXPECT_NEAR(degToRad(radToDeg(val)), val, EPS);
}

// ----------------- wrapToPi -----------------
TEST(MathUtilsTest, WrapToPiInRange) {
    EXPECT_NEAR(wrapToPi(0.5), 0.5, EPS);
    EXPECT_NEAR(wrapToPi(-M_PI / 2), -M_PI / 2, EPS);
    EXPECT_NEAR(wrapToPi(M_PI / 2), M_PI / 2, EPS);
}

TEST(MathUtilsTest, WrapToPiBoundaries) {
    EXPECT_NEAR(wrapToPi(M_PI), -M_PI, EPS);   // π wraps to -π
    EXPECT_NEAR(wrapToPi(-M_PI), -M_PI, EPS);  // stays at -π
    EXPECT_NEAR(wrapToPi(2.0 * M_PI), 0.0, EPS);
    EXPECT_NEAR(wrapToPi(-2.0 * M_PI), 0.0, EPS);
}

TEST(MathUtilsTest, WrapToPiLargeValues) {
    EXPECT_NEAR(wrapToPi(10.0 * M_PI), 0.0, EPS);
    EXPECT_NEAR(wrapToPi(-10.0 * M_PI), 0.0, EPS);
}

// ----------------- unwrap -----------------
TEST(MathUtilsTest, UnwrapContinuousIncreasing) {
    constexpr double prev = 0.9 * M_PI;
    constexpr double wrapped = -0.9 * M_PI;  // wrapped value just after π
    double unwrapped = unwrap(prev, wrapped);
    EXPECT_NEAR(unwrapped, 1.1 * M_PI, EPS); // should "continue"
}

TEST(MathUtilsTest, UnwrapContinuousDecreasing) {
    constexpr double prev = -0.9 * M_PI;
    constexpr double wrapped = 0.9 * M_PI;  // wrapped value just before -π
    double unwrapped = unwrap(prev, wrapped);
    EXPECT_NEAR(unwrapped, -1.1 * M_PI, EPS);
}

TEST(MathUtilsTest, UnwrapNoCrossing) {
    constexpr double prev = 1.0;
    constexpr double wrapped = 1.1;
    double unwrapped = unwrap(prev, wrapped);
    EXPECT_NEAR(unwrapped, 1.1, EPS);
}