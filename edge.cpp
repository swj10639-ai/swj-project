#include <sstream>
#include "edge.h"

Edge::Edge(int id, int from, int to, int length)
        : id(id), from(from), to(to), length(length),
          neighbours() {}


int Edge::getOtherVertex(int vertex) const {
    return vertex == from ? to : from;
}
int Edge::operator[](int vertex) const {
    return getOtherVertex(vertex); }

Edge *Edge::addNeighbour(Edge *neighbour) {
    neighbours.push_back(neighbour);
    return neighbours.back();
}

bool Edge::connectsTo(int vertex) const {
    return vertex == to || vertex == from;
}

int Edge::sharedVertex(const Edge *otherEdge) const {
    if (otherEdge->connectsTo(from))
        return from;
    if (otherEdge->connectsTo(to))
        return to;
    return -1;
}


bool Edge::containsNeighbour(const Edge* otherEdge) const {
    for (const Edge* neighbour : neighbours) {
        if (neighbour == otherEdge) {
            return true;
        }
    }
    return false;
}


std::string Edge::toString() const {
    std::stringstream str;
    str << "from " << from << " to " << to;
    return str.str();
}


