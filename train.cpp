
#include "train.h"


Train::Train(uint32_t id, uint32_t length, uint32_t depTime, const Edge* const startEdge): id(id),  length(length), start(depTime, startEdge){
    stops.push_back(start);
}

void Train::addStop(uint32_t arrivalTime, const Edge* const stopEdge) {
    stops.push_back(Stop(arrivalTime, stopEdge));
}


bool Train::operator==(const Train& otherTrain) const {
    return id == otherTrain.id;
}