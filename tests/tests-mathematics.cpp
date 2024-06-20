/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "gtest/gtest.h"
#include "Mathematics/grsMath.h"

TEST(testsMathFunctions, testMinValue) {

    EXPECT_DOUBLE_EQ(MIN(-4.0,-2.0), -4.0)   << "Failed to compare negatives values";
    EXPECT_DOUBLE_EQ(MIN(-4.0,0.0), -4.0)   << "Failed to compare negative value to 0";
    EXPECT_DOUBLE_EQ(MIN(0.0,2.0),  0.0)   << "Failed to compare positive value to 0";
    EXPECT_DOUBLE_EQ(MIN(2.0,4.0),  2.0)   << "Failed to compare positives values";
    EXPECT_DOUBLE_EQ(MIN(1.2,1.8),  1.2) << "Failed to compare decimal values";
    EXPECT_DOUBLE_EQ(MIN(-4.0,4.0), -4.0)   << "Failed to compare positives and negatives values";
}

TEST(testsMathFunctions, testMaxValue) {

    EXPECT_DOUBLE_EQ(MAX(-4.0,-2.0), -2.0)   << "Failed to compare negatives values";
    EXPECT_DOUBLE_EQ(MAX(-4.0,0.0),  0.0)   << "Failed to compare negative value to 0";
    EXPECT_DOUBLE_EQ(MAX(0.0,2.0),  2.0)   << "Failed to compare positive value to 0";
    EXPECT_DOUBLE_EQ(MAX(2.0,4.0),  4.0)   << "Failed to compare positives values";
    EXPECT_DOUBLE_EQ(MAX(1.2,1.8),  1.8) << "Failed to compare decimal values";
    EXPECT_DOUBLE_EQ(MAX(-4.0,4.0),  4.0)   << "Failed to compare positives and negatives values";
}

TEST(testsMathFunctions, testWrapPi) {

    EXPECT_DOUBLE_EQ(wrapPI(-1.0), -1.0) << "Failed to wrap in range negative value";
    EXPECT_DOUBLE_EQ(wrapPI( 1.0),  1.0) << "Failed to wrap in range positive value";
    EXPECT_DOUBLE_EQ(wrapPI(-4.0), -4.0 + M_2PI) << "Failed to wrap out range negative value";
    EXPECT_DOUBLE_EQ(wrapPI( 4.0),  4.0 - M_2PI) << "Failed to wrap out range positive value";
    EXPECT_DOUBLE_EQ(wrapPI( 0.0),  0.0) << "Failed to wrap null value";
}

TEST(testsMathFunctions, testWrap2Pi) {

    EXPECT_DOUBLE_EQ(wrap2PI(-4.0), -4.0 + M_2PI) << "Failed to wrap in range negative value";
    EXPECT_DOUBLE_EQ(wrap2PI( 4.0),  4.0) << "Failed to wrap in range positive value";
    EXPECT_DOUBLE_EQ(wrap2PI(-8.0), -8.0 + 2*M_2PI) << "Failed to wrap out off range negative value";
    EXPECT_DOUBLE_EQ(wrap2PI( 8.0),  8.0 - M_2PI) << "Failed to wrap out off range positive value";
    EXPECT_DOUBLE_EQ(wrap2PI( 0.0),  0.0) << "Failed to wrap null value";
}

TEST(testsMathFunctions, testIsZero) {

    EXPECT_FALSE(isZero(4.0)) << "Failed to assert positive value";
    EXPECT_FALSE(isZero(-4.0)) << "Failed to assert negative value";
    EXPECT_TRUE(isZero(0.0)) << "Failed to assert null value";
}

TEST(testsMathFunctions, testIsPositive) {

    EXPECT_FALSE(isPositive(-4.0)) << "Failed to assert negative value";
    EXPECT_FALSE(isPositive(0.0)) << "Failed to assert null value";
    EXPECT_TRUE(isPositive(4.0)) << "Failed to assert positive value";
}

TEST(testsMathFunctions, testIsNegative) {

    EXPECT_TRUE(isNegative(-4.0)) << "Failed to assert negative value";
    EXPECT_FALSE(isNegative(0.0)) << "Failed to assert null value";
    EXPECT_FALSE(isNegative(4.0)) << "Failed to assert positive value";
}

TEST(testsMathFunctions, testConstraintValue) {

    EXPECT_DOUBLE_EQ(constrainValue(2.0, 1.0, 3.0), 2.0) << "In range bounds positives";
    EXPECT_DOUBLE_EQ(constrainValue(-2.0, -3.0, -1.0), -2.0) << "In range bounds negatives";
    EXPECT_DOUBLE_EQ(constrainValue(0.0, 1.0, 3.0), 1.0) << "Under range bounds positives";
    EXPECT_DOUBLE_EQ(constrainValue(-4.0, -3.0, -1.0), -3.0) << "Under range bounds negatives";
    EXPECT_DOUBLE_EQ(constrainValue(4.0, 1.0, 3.0), 3.0) << "Over range bounds positives";
    EXPECT_DOUBLE_EQ(constrainValue(0.0, -3.0, -1.0), -1.0) << "Over range bounds negatives";
}

TEST(testsMathFunctions, testConstraintFloat) {

    EXPECT_DOUBLE_EQ(constrainFloat(2.0, 1.0, 3.0), 2.0) << "In range bounds positives";
    EXPECT_DOUBLE_EQ(constrainFloat(-2.0, -3.0, -1.0), -2.0) << "In range bounds negatives";
    EXPECT_DOUBLE_EQ(constrainFloat(0.0, 1.0, 3.0), 1.0) << "Under range bounds positives";
    EXPECT_DOUBLE_EQ(constrainFloat(-4.0, -3.0, -1.0), -3.0) << "Under range bounds negatives";
    EXPECT_DOUBLE_EQ(constrainFloat(4.0, 1.0, 3.0), 3.0) << "Over range bounds positives";
    EXPECT_DOUBLE_EQ(constrainFloat(0.0, -3.0, -1.0), -1.0) << "Over range bounds negatives";
}

TEST(testsMathFunctions, testUnwrap) {

    EXPECT_DOUBLE_EQ(unwrap(2.0, 3.0), 3.0) << "Less than PI positive jump";
    EXPECT_DOUBLE_EQ(unwrap(3.0, 2.0), 2.0) << "Less than PI negative jump";
    EXPECT_DOUBLE_EQ(unwrap((double)M_PI, (double)M_2PI), (double)M_2PI) << "Equal PI jump";
    EXPECT_DOUBLE_EQ(unwrap(2.0, 6.0), 6.0-(double)M_2PI) << "Greater than PI positive jump";
    EXPECT_DOUBLE_EQ(unwrap(6.0, 2.0), 2.0+(double)M_2PI) << "Greater than PI negative jump";
    EXPECT_DOUBLE_EQ(unwrap(0.0, 0.0), 0.0) << "Null value";
}