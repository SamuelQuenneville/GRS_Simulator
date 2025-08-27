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
#include "Mathematics/matrix3.h"
#include "Mathematics/transform.h"

class Frame {

public:
    explicit Frame(const std::string& name = "", const Frame* reference = nullptr, const Vector3<double>& position = Vector3<double>::zeros(), const Vector3<double>& orientation = Vector3<double>::zeros());
    ~Frame() = default;

    void setLoadCallback(std::function<void(Frame&)> callback);
    void computeLoad();

    void setForce(const Vector3<double>& force) const {m_origin->setForce(force);};
    void setMoment(const Vector3<double>& moment) const {m_origin->setMoment(moment);};

    [[nodiscard]] const Vector3<double>& getLocalPosition() const;
    void setLocalPosition(const Vector3<double>& position) {m_localPosition = position;};

    [[nodiscard]] const Vector3<double>& getLocalOrientation() const;
    void setLocalOrientation(const Vector3<double>& orientation) {m_localOrientation = orientation;};

    [[nodiscard]] Node& originNode() const;
    [[nodiscard]] std::string getName() const {return m_name;};

    [[nodiscard]] Vector3<double> forceBody() const { return m_frameToBody * originNode().force();}
    [[nodiscard]] Vector3<double> momentBody() const { return m_frameToBody * originNode().moment();}

    void setTransform(const Transform& newTransform);
    void setTransform(const Vector3<double>& newPosition, const Quaternion& newOrientation);
    [[nodiscard]] const Transform& transform() const;

    void setIsCenterOfMass(const bool isCenterOfMass) {m_isCenterOfMass = isCenterOfMass;};
    [[nodiscard]] bool isCenterOfMass() const {return m_isCenterOfMass;}


private:
    std::unique_ptr<Node> m_origin;
    Transform m_transform = Transform::identity();

    std::function<void(Frame&)> m_loadCallback;

    Vector3<double> m_localPosition    = Vector3<double>::zeros();
    Vector3<double> m_localOrientation = Vector3<double>::zeros();

    Matrix3<double> m_frameToBody;

    const Frame* m_referenceOriginFrame;
    std::string m_name;

    bool m_isCenterOfMass = false;
};

#endif //GRS_SIMULATOR_FRAME_H
