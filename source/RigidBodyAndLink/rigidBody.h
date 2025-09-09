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

    virtual std::map<int, Vector3<double>> externalForces();

    virtual void setInitialStates(initialStates& states) {};

    virtual void addConnection(Node& frameNode, Node& node);
    virtual void deleteConnection(Node& frameNode, Node& node);

    void updateForcesOnBody();
    void updateWorldInfos(const Vector3<double>& position, const Vector3<double>& velocity, const Vector3<double>& acceleration);

    void setCenterOfMassTransform(const Transform& newTransform) const;
    void setTransform(const Transform& newTransform) const;
    void setTransform(const Vector3<double>& newPosition, const Quaternion& newOrientation) const;
    [[nodiscard]] const Transform& transform() const;

    void setAttitude(const Vector3<double>& newAttitude);
    void setAngularVelocity(const Vector3<double>& newAngularVelocity);
    void setAngularAcceleration(const Vector3<double>& newAngularAcceleration);
    void setRotationMatrixCBodyEarth(const Matrix3<double>& newRotationMatrix);

    [[nodiscard]] const Vector3<double>& getAttitude() const;
    [[nodiscard]] const Vector3<double>& getAngularVelocity() const;
    [[nodiscard]] const Vector3<double>& getAngularAcceleration() const;
    [[nodiscard]] const Matrix3<double>& getRotationMatrixCBodyEarth() const;

    [[nodiscard]] Frame* bodyOriginFrame() const {return m_bodyOriginFrame.get();}

    [[nodiscard]] Frame* centerOfMassFrame() const;
    [[nodiscard]] Node& centerOfMass() const;
    void setCenterOfMassFrame(Frame& frame) const;

    Frame& addFrame(const std::string& name, const Frame* reference, const Vector3<double>& position, const Vector3<double>& orientation, Color::HexColor color);
    [[nodiscard]] Frame* getFrame(const std::string& name) const;
    [[nodiscard]] std::vector<Frame*> frames() const;

    std::map<Node*, std::vector<Node*>> connections;

private:
    std::unique_ptr<Frame> m_bodyOriginFrame;       // Origin of the body
    std::vector<std::unique_ptr<Frame>> m_frames;

    Vector3<double> m_attitude = Vector3<double>::zeros();
    Vector3<double> m_angularVelocity = Vector3<double>::zeros();
    Vector3<double> m_angularAcceleration = Vector3<double>::zeros();

    Matrix3<double> m_rotationMatrixCBodyEarth = Matrix3<double>::eye();
};


#endif //GRS_SIMULATOR_RIGIDBODY_H
