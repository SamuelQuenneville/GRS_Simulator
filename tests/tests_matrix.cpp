/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "gtest/gtest.h"
#include "Mathematics/math.h"

constexpr float EPS = 1e-6f;

using namespace grs;

TEST(MatrixFullTest, ConstructionAndAccess) {
    Matrix<float,3> m{1,2,3,4,5,6,7,8,9};
    EXPECT_EQ(m(0,0), 1);
    EXPECT_EQ(m(1,1), 5);
    EXPECT_EQ(m(2,2), 9);

    Matrix<float,2> z;
    EXPECT_EQ(z(0,0), 0.f);
    EXPECT_EQ(z(1,1), 0.f);

    // initializer list with fewer elements
    Matrix<float,2> m2{1};
    EXPECT_EQ(m2(0,0), 1.f);
    EXPECT_EQ(m2(0,1), 0.f);
    EXPECT_EQ(m2(1,1), 0.f);
}

TEST(MatrixFullTest, IdentityAndDiagonal) {
    auto I = Matrix<float,3>::identity();
    for (size_t i=0;i<3;i++) {
        EXPECT_EQ(I(i,i), 1.f);
        for (size_t j=0;j<3;j++) if(i!=j) EXPECT_EQ(I(i,j), 0.f);
    }

    auto D = Matrix<float,3>::diagonal({1.f,2.f,3.f});
    EXPECT_EQ(D(0,0), 1.f);
    EXPECT_EQ(D(1,1), 2.f);
    EXPECT_EQ(D(2,2), 3.f);
    EXPECT_EQ(D(0,1), 0.f);
}

TEST(MatrixFullTest, AdditionSubtractionScalar) {
    Matrix<float,2> A{1,2,3,4};
    Matrix<float,2> B{4,3,2,1};

    auto C = A + B;
    EXPECT_EQ(C(0,0), 5);
    EXPECT_EQ(C(0,1), 5);
    EXPECT_EQ(C(1,0), 5);
    EXPECT_EQ(C(1,1), 5);

    auto D = B - A;
    EXPECT_EQ(D(0,0), 3);
    EXPECT_EQ(D(0,1), 1);
    EXPECT_EQ(D(1,0), -1);
    EXPECT_EQ(D(1,1), -3);

    auto E = A * 2.f;
    EXPECT_EQ(E(0,0), 2);
    EXPECT_EQ(E(1,1), 8);

    auto F = 2.f * A;
    EXPECT_EQ(F(0,1), 4);
    EXPECT_EQ(F(1,0), 6);

    auto G = A / 2.f;
    EXPECT_EQ(G(0,0), 0.5f);
    EXPECT_EQ(G(1,1), 2.f);
}

TEST(MatrixFullTest, MatrixVectorMultiplication) {
    const Matrix<float,3> R = Matrix<float,3>::identity();
    const Vector<float,3> v{1,2,3};
    auto w = R * v;
    EXPECT_EQ(w[0], 1);
    EXPECT_EQ(w[1], 2);
    EXPECT_EQ(w[2], 3);

    // 4x4 homogeneous multiplication
    Matrix<float,4> T = Matrix<float,4>::identity();
    const Vector<float,3> p{1,2,3};
    auto q = T * p;
    EXPECT_EQ(q[0], 1);
    EXPECT_EQ(q[1], 2);
    EXPECT_EQ(q[2], 3);

    T(0,3) = 10; // translation
    q = T * p;
    EXPECT_EQ(q[0], 11);
}

TEST(MatrixFullTest, MatrixMatrixMultiplication) {
    const Matrix<float,2> A{1,2,3,4};
    const Matrix<float,2> B{2,0,1,2};
    auto C = A * B;
    EXPECT_EQ(C(0,0), 4);
    EXPECT_EQ(C(0,1), 4);
    EXPECT_EQ(C(1,0), 10);
    EXPECT_EQ(C(1,1), 8);
}

TEST(MatrixFullTest, Transpose) {
    const Matrix<float,2> A{1,2,3,4};
    auto AT = A.transpose();
    EXPECT_EQ(AT(0,1), 3);
    EXPECT_EQ(AT(1,0), 2);
}

TEST(MatrixFullTest, DeterminantAndInverse) {
    const Matrix<float,2> A{4,7,2,6};
    EXPECT_NEAR(A.determinant(), 10.f, EPS);

    auto Ainv = A.inverse();
    EXPECT_NEAR(Ainv(0,0), 0.6f, EPS);
    EXPECT_NEAR(Ainv(0,1), -0.7f, EPS);
    EXPECT_NEAR(Ainv(1,0), -0.2f, EPS);
    EXPECT_NEAR(Ainv(1,1), 0.4f, EPS);

    const Matrix<float,3> B{1,2,3,0,1,4,5,6,0};
    EXPECT_NEAR(B.determinant(), 1.f, EPS);
    const auto Binv = B.inverse();
    auto I = B * Binv;
    for(size_t i=0;i<3;i++)
        for(size_t j=0;j<3;j++)
            EXPECT_NEAR(I(i,j), (i==j?1.f:0.f), EPS);

    const Matrix<double,4> C{4,7,2,3,0,5,0,1,0,0,3,0,2,0,1,1};
    const Matrix<double,4> CinvExpected = {1.25f,-1.75f,-0.1666666f,-2.0f,0.5f,-0.5f,0.f,-1.0f,0.f,0.f,0.33333333f,0.f,-2.5f,3.5f,0.f,5.0f};
    const auto Cinv = C.inverse();

    for(size_t i=0;i<3;i++)
        for(size_t j=0;j<3;j++)
            EXPECT_NEAR(Cinv(i,j), CinvExpected(i,j), EPS);
}

TEST(MatrixFullTest, UpperLeft3x3) {
    const Matrix<float,4> M{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    auto U = M.upperLeft3x3();
    EXPECT_EQ(U(0,0),1);
    EXPECT_EQ(U(2,2),11);
}

TEST(MatrixFullTest, RotationMatrices) {
    const auto Rx = rotationX(M_PI_2f);
    Vector<float,3> v{0,1,0};
    auto v_rot = Rx * v;
    EXPECT_NEAR(v_rot[1], 0.f, EPS);
    EXPECT_NEAR(v_rot[2], -1.f, EPS);

    const auto Ry = rotationY(M_PI_2f);
    v = Vector<float,3>{1,0,0};
    v_rot = Ry * v;
    EXPECT_NEAR(v_rot[0], 0.f, EPS);
    EXPECT_NEAR(v_rot[2], 1.f, EPS);

    const auto Rz = rotationZ(M_PI_2f);
    v = Vector<float,3>{1,0,0};
    v_rot = Rz * v;
    EXPECT_NEAR(v_rot[0], 0.f, EPS);
    EXPECT_NEAR(v_rot[1], -1.f, EPS);
}

TEST(MatrixFullTest, EulerToMatrix) {
    const auto R = eulerToMat(0.f,0.f,M_PI_2f);
    const Vector<float,3> v{1,0,0};
    auto v_rot = R * v;
    EXPECT_NEAR(v_rot[0], 0.f, EPS);
    EXPECT_NEAR(v_rot[1], -1.f, EPS);

    Vector<float,3> euler{0.f,0.f,M_PI_2f};
    const auto R2 = eulerToMat(euler);
    v_rot = R2 * v;
    EXPECT_NEAR(v_rot[0], 0.f, EPS);
    EXPECT_NEAR(v_rot[1], -1.f, EPS);
}

TEST(MatrixFullTest, MakeRotationAndTranslation4x4) {
    const Vector<float,3> axis{0,0,1};
    const auto R = makeRotation(axis, M_PI_2f);
    const Vector<float,3> v{1,0,0};
    auto v_rot = R * v;
    EXPECT_NEAR(v_rot[0], 0.f, EPS);
    EXPECT_NEAR(v_rot[1], 1.f, EPS);

    const Vector<float,3> t{1,2,3};
    const auto T = makeTranslation(t);
    v_rot = T * Vector<float,3>{0,0,0};
    EXPECT_NEAR(v_rot[0], 1.f, EPS);
    EXPECT_NEAR(v_rot[1], 2.f, EPS);
    EXPECT_NEAR(v_rot[2], 3.f, EPS);
}