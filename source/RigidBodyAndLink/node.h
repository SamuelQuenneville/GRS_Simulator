/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_NODE_H
#define GRS_SIMULATOR_NODE_H

#include "Mathematics/math.h"

class Node {

public:
    Node();
    ~Node() = default;

    [[nodiscard]] int getId() const;

    void update(const grs::Vec3d& newPosition, const grs::Vec3d& newVelocity, const grs::Vec3d& newForce, const grs::Vec3d& newMoment);

    [[nodiscard]] const grs::Vec3d& position() const;
    void setPosition(const grs::Vec3d& newPosition);

    [[nodiscard]] const grs::Vec3d& velocity() const;
    void setVelocity(const grs::Vec3d& newVelocity);

    [[nodiscard]] const grs::Vec3d& acceleration() const;
    void setAcceleration(const grs::Vec3d& newAcceleration);

    [[nodiscard]] const grs::Vec3d& force() const;
    void setForce(const grs::Vec3d& newForce);

    [[nodiscard]] const grs::Vec3d& moment() const;
    void setMoment(const grs::Vec3d& newMoment);

private:
    int m_id;
    static int m_idCounter;

    grs::Vec3d m_position     = grs::Vec3d::zeros();        // Own reference frame
    grs::Vec3d m_velocity     = grs::Vec3d::zeros();        // Own reference frame
    grs::Vec3d m_acceleration = grs::Vec3d::zeros();        // Own reference frame

    grs::Vec3d m_force        = grs::Vec3d::zeros();        // Own reference frame
    grs::Vec3d m_moment       = grs::Vec3d::zeros();        // Own reference frame
};

#endif //GRS_SIMULATOR_NODE_H