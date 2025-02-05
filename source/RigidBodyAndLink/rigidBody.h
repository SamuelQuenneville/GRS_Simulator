/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_RIGIDBODY_H
#define GRS_SIMULATOR_RIGIDBODY_H

#include <map>
#include <memory>
#include <algorithm>

#include "Definitions/worldPhysics.h"
#include "Definitions/configurationDefinitions.h"
#include "Mathematics/grsMath.h"
#include "Mathematics/matrix3.h"
#include "Mathematics/transform.h"
#include "node.h"

#include "Log/logger.h"

class RigidBody {

public:
    RigidBody();
    virtual ~RigidBody() = default;

    virtual void update(double dt) {};
    virtual Vector3<double> calcExternalForce();
    virtual std::map<int, Vector3<double>> externalForces();

    virtual void setInitialStates(initialStates& states) {};

    virtual void addConnection(Node& node);
    virtual void deleteConnection(Node& node);

    [[nodiscard]] virtual Node& node();

    void setTransform(const Transform& newTransform);
    void setTransform(const Vector3<double>& newPosition, const Quaternion& newOrientation);
    [[nodiscard]] const Transform& transform() const;

    void setVelocity(const Vector3<double>& newVelocity) const;

    void setAttitude(const Vector3<double>& newAttitude);
    void setAngularVelocity(const Vector3<double>& newAngularVelocity);

    [[nodiscard]] const Vector3<double>& getAttitude() const;
    [[nodiscard]] const Vector3<double>& getAngularVelocity() const;

    std::unique_ptr<Node> centerOfMass;
    std::vector<Node*> connectedNode;

private:
    Transform m_transform = Transform::identity();

    Vector3<double> m_attitude{0.0, 0.0, 0.0};        // Earth reference frame
    Vector3<double> m_angularVelocity{0.0,0.0,0.0};   // Earth reference frame
};


#endif //GRS_SIMULATOR_RIGIDBODY_H
