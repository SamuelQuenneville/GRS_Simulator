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

    [[nodiscard]] int getId() const;

    void update(Vector3<double> newPosition, Vector3<double> newVelocity, Vector3<double> newForce, Vector3<double> newMoment);

    [[nodiscard]] const Vector3<double> &position() const;
    void setPosition(const Vector3<double> &newPosition);

    [[nodiscard]] const Vector3<double> &velocity() const;
    void setVelocity(const Vector3<double> &newVelocity);

    [[nodiscard]] const Vector3<double> &force() const;
    void setForce(const Vector3<double> &newForce);

    [[nodiscard]] const Vector3<double> &moment() const;
    void setMoment(const Vector3<double> &newMoment);

private:
    int m_id;
    static int m_idCounter;

    Vector3<double> m_position{0,0,0};  // Earth reference frame
    Vector3<double> m_velocity{0,0,0};  // Earth reference frame
    Vector3<double> m_force{0,0,0};     // Earth reference frame
    Vector3<double> m_moment{0,0,0};    // Earth reference frame
};

#endif //GRS_SIMULATOR_NODE_H