
#include "train.h"


//Train类的构造函数
Train::Train(uint32_t id, uint32_t length, uint32_t depTime, const Edge* const startEdge): id(id),  length(length), start(depTime, startEdge){
    stops.push_back(start);   //提供的出发时间和起始边创建一个Stop对象，并将其添加到stops列表中。
}

//添加停靠点。
void Train::addStop(uint32_t arrivalTime, const Edge* const stopEdge) {
    stops.push_back(Stop(arrivalTime, stopEdge));
}


bool Train::operator==(const Train& otherTrain) const { //用于比较两个Train对象是否相等。它通过比较两个对象的id成员变量来判断它们是否相等。
    return id == otherTrain.id;
}