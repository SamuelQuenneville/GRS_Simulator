/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_RIGIDBODY_H
#define GRS_SIMULATOR_RIGIDBODY_H

#include <vector>
#include <map>
#include <memory>
#include <algorithm>

#include "Definitions/worldPhysics.h"
#include "Definitions/configurationDefinitions.h"
#include "Mathematics/grsMath.h"
#include "Mathematics/matrix3.h"
#include "node.h"

class RigidBody {

public:
    RigidBody();
    virtual ~RigidBody() = default;

    enum rigidBodyType {
        AIRCRAFT,
        PAYLOAD
    };

    virtual void update(double dt) = 0;
    virtual Vector3<double> calcExternalForce();

    virtual void setInitialStates(initialStates& states) = 0;

    virtual void addConnection(Node& node);
    virtual void deleteConnection(Node& node);

    [[nodiscard]] virtual Node& node();

    std::unique_ptr<Node> centerOfMass;
    std::vector<Node*> connectedNode;
};


#endif //GRS_SIMULATOR_RIGIDBODY_H
