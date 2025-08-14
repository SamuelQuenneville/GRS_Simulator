/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_FRAME_H
#define GRS_SIMULATOR_FRAME_H

#include <memory>

#include "node.h"

class Frame {

public:
    explicit Frame(const Frame* reference = nullptr, const Vector3<double>& position = Vector3<double>::zeros(), const Vector3<double>& orientation = Vector3<double>::zeros());
    ~Frame() = default;

    [[nodiscard]] const Vector3<double>& getLocalPosition() const;
    void setLocalPosition(const Vector3<double>& position) {m_localPosition = position;};

    [[nodiscard]] const Vector3<double>& getLocalOrientation() const;
    void setLocalOrientation(const Vector3<double>& orientation) {m_localOrientation = orientation;};

    [[nodiscard]] const std::shared_ptr<Node>& origin() const;

private:
    std::shared_ptr<Node> m_origin;

    Vector3<double> m_localPosition = Vector3<double>::zeros();
    Vector3<double> m_localOrientation = Vector3<double>::zeros();

    const Frame* m_reference = nullptr;
};

#endif //GRS_SIMULATOR_FRAME_H
