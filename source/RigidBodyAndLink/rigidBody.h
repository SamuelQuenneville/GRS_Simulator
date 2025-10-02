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

#include "Definitions/configurationDefinitions.h"
#include "frame.h"
#include "Log/logger.h"

class RigidBody {

public:
    RigidBody();
    virtual ~RigidBody() = default;

    virtual void update(double dt) {};

    virtual std::map<int, grs::Vec3d> externalForces();

    virtual void setInitialStates(initialStates& states) {};

    virtual void addConnection(Node& frameNode, Node& node);
    virtual void deleteConnection(Node& frameNode, Node& node);

    void updateForcesOnBody();
    void updateWorldInfos(const grs::Vec3d& position, const grs::Vec3d& velocity, const grs::Vec3d& acceleration) const;

    void setCenterOfMassTransform(const grs::TransformD& newTransform) const;
    void setTransform(const grs::TransformD& newTransform) const;
    void setTransform(const grs::Vec3d& newPosition, const grs::QuaternionD& newOrientation) const;
    [[nodiscard]] const grs::TransformD& transform() const;

    void setAttitude(const grs::Vec3d& newAttitude);
    void setAngularVelocity(const grs::Vec3d& newAngularVelocity);
    void setAngularAcceleration(const grs::Vec3d& newAngularAcceleration);
    void setRotationMatrixCBodyEarth(const grs::Matrix3d& newRotationMatrix);

    [[nodiscard]] const grs::Vec3d& getAttitude() const;
    [[nodiscard]] const grs::Vec3d& getAngularVelocity() const;
    [[nodiscard]] const grs::Vec3d& getAngularAcceleration() const;
    [[nodiscard]] const grs::Matrix3d& getRotationMatrixCBodyEarth() const;

    [[nodiscard]] Frame* bodyOriginFrame() const {return m_bodyOriginFrame.get();}

    [[nodiscard]] Frame* centerOfMassFrame() const;
    [[nodiscard]] Node& centerOfMass() const;
    void setCenterOfMassFrame(Frame& frame) const;

    Frame& addFrame(const std::string& name, const Frame* reference, const grs::Vec3d& position, const grs::Vec3d& orientation, Color::HexColor color);
    [[nodiscard]] Frame* getFrame(const std::string& name) const;
    [[nodiscard]] std::vector<Frame*> frames() const;

    std::map<Node*, std::vector<Node*>> connections;

private:
    std::unique_ptr<Frame> m_bodyOriginFrame;       // Origin of the body
    std::vector<std::unique_ptr<Frame>> m_frames;

    grs::Vec3d m_attitude = grs::Vec3d::zeros();
    grs::Vec3d m_angularVelocity = grs::Vec3d::zeros();
    grs::Vec3d m_angularAcceleration = grs::Vec3d::zeros();

    grs::Matrix3d m_rotationMatrixCBodyEarth = grs::Matrix3d::identity();
};


#endif //GRS_SIMULATOR_RIGIDBODY_H
