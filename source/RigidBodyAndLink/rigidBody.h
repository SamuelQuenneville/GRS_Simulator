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
#include "frame.h"

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

    void updateWorldInfos(const Vector3<double>& position, const Vector3<double>& velocity, const Vector3<double>& acceleration) const;

    void setTransform(const Transform& newTransform);
    void setTransform(const Vector3<double>& newPosition, const Quaternion& newOrientation);
    [[nodiscard]] const Transform& transform() const;

    void setAttitude(const Vector3<double>& newAttitude);
    void setAngularVelocity(const Vector3<double>& newAngularVelocity);

    [[nodiscard]] const Vector3<double>& getAttitude() const;
    [[nodiscard]] const Vector3<double>& getAngularVelocity() const;

    [[nodiscard]] Frame* bodyOriginFrame() const {return m_bodyOriginFrame.get();}

    [[nodiscard]] Frame* centerOfMassFrame() const;
    [[nodiscard]] Node& centerOfMass() const;
    static void setCenterOfMassFrame(Frame& frame) {frame.isCenterOfMass = true;}

    Frame& addFrame(const std::string& name, const Frame* reference, const Vector3<double>& position, const Vector3<double>& orientation);
    [[nodiscard]] Frame* getFrame(const std::string& name) const;

    std::vector<Node*> connectedNode;

private:
    Transform m_transform = Transform::identity();

    std::unique_ptr<Frame> m_bodyOriginFrame;       // Origin of the body
    std::vector<std::unique_ptr<Frame>> m_frames;

    Vector3<double> m_attitude = Vector3<double>::zeros();          // Earth reference frame
    Vector3<double> m_angularVelocity = Vector3<double>::zeros();   // Earth reference frame
};


#endif //GRS_SIMULATOR_RIGIDBODY_H
