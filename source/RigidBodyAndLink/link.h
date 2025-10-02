/*
 * GRS Simulation Environment
 * Samuel Quenneville (samuel.quenneville@usherbrooke.ca)
 *
 * Université de Sherbrooke
 * Createk Innovation Lab
 */

#ifndef GRS_SIMULATOR_LINK_H
#define GRS_SIMULATOR_LINK_H

#include <vector>
#include <map>
#include <memory>

#include "node.h"

class Link {

public:
    Link();
    virtual ~Link() = default;

    enum linkType {
        RIGID,
        FLEXIBLE
    };

    virtual void update(double dt) = 0;

    virtual void addConnection(Node& linkEnd, Node& node);
    virtual void deleteConnection(Node& linkEnd, Node& node);

    [[nodiscard]] virtual Node& startPointNode() const;
    [[nodiscard]] virtual Node& endPointNode() const;

    std::unique_ptr<Node> startPoint;
    std::unique_ptr<Node> endPoint;

    std::map<Node*, std::vector<Node*>> connections;
};


#endif //GRS_SIMULATOR_LINK_H
