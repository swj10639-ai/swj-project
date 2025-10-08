#ifndef SATCBS_TRAIN_TRAIN_H
#define SATCBS_TRAIN_TRAIN_H

#include "edge.h"
#include <vector>
#include <cstdint>

struct Stop {
    uint32_t arrivalTime;
    const Edge* stopEdge;

    Stop(uint32_t arrivalTime, const Edge* const stopEdge): arrivalTime(arrivalTime), stopEdge(stopEdge){};
};


class Train {

public:
    const uint32_t id, length;
    Stop start;

    std::vector<Stop> stops;
    std::vector<std::vector<const Edge*>> paths;

    Train(uint32_t id, uint32_t length, uint32_t depTime, const Edge* const start);

    void addStop(uint32_t arrivalTime, const Edge* const stopEdge);

    bool operator==(const Train& otherTrain) const;
};


#endif //SATCBS_TRAIN_TRAIN_H
