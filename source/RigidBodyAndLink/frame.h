/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FRAME_H
#define GRS_SIMULATOR_FRAME_H

#include <functional>
#include <memory>

#include "node.h"
#include "Mathematics/color.h"
#include "Definitions/renderObjectDefinitions.h"
#include "Log/logger.h"

class Frame {

public:
    explicit Frame(const std::string& name = "", const Frame* reference = nullptr, const grs::Vec3d& position = grs::Vec3d::zeros(), const grs::Vec3d& orientation = grs::Vec3d::zeros(), Color::HexColor forceColor = Color::HexColor::YELLOW);
    ~Frame() = default;

    void setLoadCallback(std::function<void(Frame&)> callback);
    void computeLoad();

    void writeLog(double simTime) const;

    void setForce(const grs::Vec3d& force) const {m_origin->setForce(force);};
    void setMoment(const grs::Vec3d& moment) const {m_origin->setMoment(moment);};

    [[nodiscard]] const grs::Vec3d& getLocalPosition() const;
    void setLocalPosition(const grs::Vec3d& position) {m_localPosition = position;};

    [[nodiscard]] const grs::Vec3d& getLocalOrientation() const;
    void setLocalOrientation(const grs::Vec3d& orientation) {m_localOrientation = orientation;};

    [[nodiscard]] Node& originNode() const;
    [[nodiscard]] std::string getName() const {return m_name;};

    [[nodiscard]] grs::Vec3d forceBody() const {return m_frameToBody * originNode().force();}
    [[nodiscard]] grs::Vec3d momentBody() const {return m_frameToBody * originNode().moment();}

    void setTransform(const grs::TransformD& newTransform);
    void setTransform(const grs::Vec3d& newPosition, const grs::QuaternionD& newOrientation);
    [[nodiscard]] const grs::TransformD& transform() const;

    void setIsCenterOfMass(const bool isCenterOfMass) {m_isCenterOfMass = isCenterOfMass;};
    [[nodiscard]] bool isCenterOfMass() const {return m_isCenterOfMass;}

    [[nodiscard]] Color::HexColor getForceColor() const {return m_forceColor;}

    [[nodiscard]] frameRenderData packRenderData() const;


private:
    std::unique_ptr<Node> m_origin;
    grs::TransformD m_transform = grs::TransformD::identity();

    std::function<void(Frame&)> m_loadCallback;

    grs::Vec3d m_localPosition    = grs::Vec3d::zeros();
    grs::Vec3d m_localOrientation = grs::Vec3d::zeros();

    grs::Matrix3d m_frameToBody{};

    const Frame* m_referenceOriginFrame;
    std::string m_name;
    Color::HexColor m_forceColor;

    bool m_isCenterOfMass = false;
};

#endif //GRS_SIMULATOR_FRAME_H
