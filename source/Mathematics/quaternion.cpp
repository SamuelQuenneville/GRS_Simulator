/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#include "quaternion.h"

std::string Quaternion::toString() const {
    return "Quaternion(" + std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z) + "," + std::to_string(w) + ")";
}

void Quaternion::m_initWithEulerAngles(double angleX, double angleY, double angleZ) {
    double angle = angleX * 0.5;
    const double sinX = std::sin(angle);
    const double cosX = std::cos(angle);

    angle = angleY * 0.5;
    const double sinY = std::sin(angle);
    const double cosY = std::cos(angle);

    angle = angleZ * 0.5;
    const double sinZ = std::sin(angle);
    const double cosZ = std::cos(angle);

    const double cosYcosZ = cosY * cosZ;
    const double sinYcosZ = sinY * cosZ;
    const double cosYsinZ = cosY * sinZ;
    const double sinYsinZ = sinY * sinZ;

    x = sinX * cosYcosZ - cosX * sinYsinZ;
    y = cosX * sinYcosZ + sinX * cosYsinZ;
    z = cosX * cosYsinZ - sinX * sinYcosZ;
    w = cosX * cosYcosZ + sinX * sinYsinZ;

    // Normalize the quaternion
    normalize();
}

Quaternion::Quaternion()
    : x(0.0), y(0.0), z(0.0), w(0.0)
{

}

Quaternion::Quaternion(double newX, double newY, double newZ, double newW)
    : x(newX), y(newY), z(newZ), w(newW)
{

}

Quaternion::Quaternion(const Vector3<double>& vector, double newW)
    : x(vector.x()), y(vector.y()), z(vector.z()), w(newW)
{

}

Quaternion::Quaternion(const Matrix3<double>& matrix) {

    // Get the trace of the matrix
    double trace = matrix.getTrace();

    double r;
    double s;

    if (trace < 0.0) {
        if (matrix[1][1] > matrix[0][0]) {
            if(matrix[2][2] > matrix[1][1]) {
                r = std::sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + 1.0);
                s = 0.5 / r;

                // Compute the quaternion
                x = (matrix[2][0] + matrix[0][2]) * s;
                y = (matrix[1][2] + matrix[2][1]) * s;
                z = 0.5 * r;
                w = (matrix[1][0] - matrix[0][1]) * s;
            }
            else {
                r = std::sqrt(matrix[1][1] - matrix[2][2] - matrix[0][0] + 1.0);
                s = 0.5 / r;

                // Compute the quaternion
                x = (matrix[0][1] + matrix[1][0]) * s;
                y = 0.5 * r;
                z = (matrix[1][2] + matrix[2][1]) * s;
                w = (matrix[0][2] - matrix[2][0]) * s;
            }
        }
        else if (matrix[2][2] > matrix[0][0]) {
            r = std::sqrt(matrix[2][2] - matrix[0][0] - matrix[1][1] + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            x = (matrix[2][0] + matrix[0][2]) * s;
            y = (matrix[1][2] + matrix[2][1]) * s;
            z = 0.5 * r;
            w = (matrix[1][0] - matrix[0][1]) * s;
        }
        else {
            r = std::sqrt(matrix[0][0] - matrix[1][1] - matrix[2][2] + 1.0);
            s = 0.5 / r;

            // Compute the quaternion
            x = 0.5 * r;
            y = (matrix[0][1] + matrix[1][0]) * s;
            z = (matrix[2][0] + matrix[0][2]) * s;
            w = (matrix[2][1] - matrix[1][2]) * s;
        }
    }
    else {
        r = std::sqrt(trace + 1.0);
        s = 0.5 / r;

        // Compute the quaternion
        x = (matrix[2][1] - matrix[1][2]) * s;
        y = (matrix[0][2] - matrix[2][0]) * s;
        z = (matrix[1][0] - matrix[0][1]) * s;
        w = 0.5 * r;
    }
}

void Quaternion::setAllValues(double newX, double newY, double newZ, double newW) {
    x = newX;
    y = newY;
    z = newZ;
    w = newW;
}

void Quaternion::setToZero() {
    setAllValues(0,0,0,0);
}

void Quaternion::setToIdentity() {
    setAllValues(0,0,0,1);
}

Vector3<double> Quaternion::getVector() const {
    return {x,y,z};
}

double Quaternion::lengthSquare() const {
    return x*x + y*y + z*z + w*w;
}

double Quaternion::length() const {
    return std::sqrt(lengthSquare());
}

void Quaternion::normalize() {
    double len = length();
    if (len != 0) {
        x /= len;
        y /= len;
        z /= len;
        w /= len;
    } else {
        throw std::invalid_argument("Can't normalize zero quaternion"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
    }
}

void Quaternion::inverse() {
    x = -x;
    y = -y;
    z = -z;
}

Quaternion Quaternion::getUnit() const {
    double len = length();

    if (len != 0) {
        return {x / len, y / len, z / len, w / len};
    } else {
        throw std::invalid_argument("Can't normalize zero quaternion"+std::string(__FILE__)+" line: "+std::to_string(__LINE__));
    }
}

Quaternion Quaternion::getConjugate() const {
    return {-x, -y, -z, w};
}

Quaternion Quaternion::getInverse() const {
    return {-x, -y, -z, w};
}

Matrix3<double> Quaternion::getMatrix() const {

    double nQ = x*x + y*y + z*z + w*w;
    double s = 0.0;

    if (nQ > 0.0) {
        s = 2.0 / nQ;
    }

    // Computations used for optimization (fewer multiplications)
    double xs = x*s;
    double ys = y*s;
    double zs = z*s;
    double wxs = w*xs;
    double wys = w*ys;
    double wzs = w*zs;
    double xxs = x*xs;
    double xys = x*ys;
    double xzs = x*zs;
    double yys = y*ys;
    double yzs = y*zs;
    double zzs = z*zs;

    // Create the matrix corresponding to the quaternion
    return Matrix3<double>{{1.0 - yys - zzs, xys-wzs, xzs + wys},
                           {xys + wzs, 1.0 - xxs - zzs, yzs-wxs},
                           {xzs-wys, yzs + wxs, 1.0 - xxs - yys}};
}

Quaternion Quaternion::identity() {
    return {0,0,0,1};
}

Quaternion Quaternion::fromEulerAngles(double angleX, double angleY, double angleZ) {
    Quaternion quaternion;
    quaternion.m_initWithEulerAngles(angleX, angleY, angleZ);

    return quaternion;
}

Quaternion Quaternion::fromEulerAngles(const Vector3<double>& eulerAngles) {
    Quaternion quaternion;
    quaternion.m_initWithEulerAngles(eulerAngles.x(), eulerAngles.y(), eulerAngles.z());

    return quaternion;
}

double Quaternion::dot(const Quaternion& quaternion) const {
    return (x*quaternion.x + y*quaternion.y + z*quaternion.z + w*quaternion.w);
}

void Quaternion::getRotationAngleAxis(double& angle, Vector3<double>& axis) const {

    // Compute the rotation angle
    angle = std::acos(w) * 2.0;

    // Compute the 3D rotation axis
    Vector3 rotationAxis(x, y, z);

    // Normalize the rotation axis
    rotationAxis = rotationAxis.normalize();

    // Set the rotation axis values
    axis.setAllValues(rotationAxis.x(), rotationAxis.y(), rotationAxis.z());
}

bool Quaternion::isFinite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z) && std::isfinite(w);
}

bool Quaternion::isUnit() const {
    const double length = std::sqrt(x*x + y*y + z*z + w*w);
    const double tolerance = 1e-5f;
    return std::abs(length - 1.0) < tolerance;
}

bool Quaternion::isValid() const {
    return isFinite() && isUnit();
}

Quaternion Quaternion::slerp(const Quaternion &quaternion1, const Quaternion& quaternion2, double t) {
    assert(t >= 0.0 && t <= 1.0);

    double invert = 1.0;

    // Compute cos(theta) using the quaternion scalar product
    double cosineTheta = quaternion1.dot(quaternion2);

    // Take care of the sign of cosineTheta
    if (cosineTheta < 0.0) {
        cosineTheta = -cosineTheta;
        invert = -1.0;
    }

    // Because of precision, if cos(theta) is nearly 1,
    // therefore theta is nearly 0 and we can write
    // sin((1-t)*theta) as (1-t) and sin(t*theta) as t
    const double epsilon = 0.00001;
    if(1-cosineTheta < epsilon) {
        return quaternion1 * (1.0-t) + quaternion2 * (t * invert);
    }

    // Compute the theta angle
    double theta = std::acos(cosineTheta);

    // Compute sin(theta)
    double sineTheta = std::sin(theta);

    // Compute the two coefficients that are in the spherical linear interpolation formula
    double coeff1 = std::sin((1.0-t)*theta) / sineTheta;
    double coeff2 = std::sin(t*theta) / sineTheta * invert;

    // Compute and return the interpolated quaternion
    return quaternion1 * coeff1 + quaternion2 * coeff2;
}

Quaternion Quaternion::operator+(const Quaternion& quaternion) const {
    return {x + quaternion.x, y + quaternion.y, z + quaternion.z, w + quaternion.w};
}

Quaternion Quaternion::operator-(const Quaternion& quaternion) const {
    return {x - quaternion.x, y - quaternion.y, z - quaternion.z, w - quaternion.w};
}

Quaternion &Quaternion::operator+=(const Quaternion& quaternion) {
    x += quaternion.x;
    y += quaternion.y;
    z += quaternion.z;
    w += quaternion.w;
    return *this;
}

Quaternion &Quaternion::operator-=(const Quaternion& quaternion) {
    x -= quaternion.x;
    y -= quaternion.y;
    z -= quaternion.z;
    w -= quaternion.w;
    return *this;
}

Quaternion Quaternion::operator*(double rhs) const {
    return {rhs * x, rhs * y, rhs * z, rhs * w};
}

Quaternion Quaternion::operator*(const Quaternion& quaternion) const {
    /* The following code is equivalent to this
     * return Quaternion(w * quaternion.w - getVectorV().dot(quaternion.getVectorV()),
     * w * quaternion.getVectorV() + quaternion.w * getVectorV() +
     * getVectorV().cross(quaternion.getVectorV()));
     */

    return {w * quaternion.x + quaternion.w * x + y * quaternion.z - z * quaternion.y,
            w * quaternion.y + quaternion.w * y + z * quaternion.x - x * quaternion.z,
            w * quaternion.z + quaternion.w * z + x * quaternion.y - y * quaternion.x,
            w * quaternion.w - x * quaternion.x - y * quaternion.y - z * quaternion.z};

}

Vector3<double> Quaternion::operator*(const Vector3<double>& point) const {
    /* The following code is equivalent to this
     * Quaternion p(point.x, point.y, point.z, 0.0);
     * return (((*this) * p) * getConjugate()).getVectorV();
    */

    const double prodX = w * point.x() + y * point.z() - z * point.y();
    const double prodY = w * point.y() + z * point.x() - x * point.z();
    const double prodZ = w * point.z() + x * point.y() - y * point.x();
    const double prodW = -x * point.x() - y * point.y() - z * point.z();
    return {w * prodX - prodY * z + prodZ * y - prodW * x,
            w * prodY - prodZ * x + prodX * z - prodW * y,
            w * prodZ - prodX * y + prodY * x - prodW * z};
}

bool Quaternion::operator==(const Quaternion& quaternion) const {
    return (x == quaternion.x && y == quaternion.y && z == quaternion.z && w == quaternion.w);
}
