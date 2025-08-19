/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_NODE_H
#define GRS_SIMULATOR_NODE_H

#include "Mathematics/vector3.h"

class Node {

public:
    Node();
    ~Node() = default;

    [[nodiscard]] int getId() const;

    void update(const Vector3<double>& newPosition, const Vector3<double>& newVelocity, const Vector3<double>& newForce, const Vector3<double>& newMoment);

    [[nodiscard]] const Vector3<double>& position() const;
    void setPosition(const Vector3<double>& newPosition);

    [[nodiscard]] const Vector3<double>& velocity() const;
    void setVelocity(const Vector3<double>& newVelocity);

    [[nodiscard]] const Vector3<double>& acceleration() const;
    void setAcceleration(const Vector3<double>& newAcceleration);

    [[nodiscard]] const Vector3<double>& force() const;
    void setForce(const Vector3<double>& newForce);

    [[nodiscard]] const Vector3<double>& moment() const;
    void setMoment(const Vector3<double>& newMoment);

private:
    int m_id;
    static int m_idCounter;

    Vector3<double> m_position     = Vector3<double>::zeros();        // Own reference frame
    Vector3<double> m_velocity     = Vector3<double>::zeros();        // Own reference frame
    Vector3<double> m_acceleration = Vector3<double>::zeros();        // Own reference frame

    Vector3<double> m_force        = Vector3<double>::zeros();        // Own reference frame
    Vector3<double> m_moment       = Vector3<double>::zeros();        // Own reference frame
};

#endif //GRS_SIMULATOR_NODE_H