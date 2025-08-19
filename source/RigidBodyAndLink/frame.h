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

    bool isCenterOfMass = false;

private:
    std::unique_ptr<Node> m_origin;

    std::function<void(Frame&)> m_loadCallback;

    Vector3<double> m_localPosition    = Vector3<double>::zeros();
    Vector3<double> m_localOrientation = Vector3<double>::zeros();

    const Frame* m_referenceOriginFrame;
    std::string m_name;
};

#endif //GRS_SIMULATOR_FRAME_H
