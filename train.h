#ifndef SATCBS_TRAIN_TRAIN_H
#define SATCBS_TRAIN_TRAIN_H

#include "edge.h"
#include <vector>
#include <cstdint>

struct Stop {//在每个时间步 到了 哪条边
    uint32_t arrivalTime; //到达时间
    const Edge* stopEdge; //停靠边的指针,指向停靠的边

    Stop(uint32_t arrivalTime, const Edge* const stopEdge): arrivalTime(arrivalTime), stopEdge(stopEdge){}; //用于初始化 Stop 结构体的成员。它接受到达时间和停靠边的指针作为参数，并使用成员初始化列表将这些值赋给对应的成员变量。
};


class Train {//火车类，目的地呢？

public:
    const uint32_t id, length;//火车的id，和长度
    Stop start;   //起始停靠点

    std::vector<Stop> stops;  //停靠点列表（在每个时间步 到了 哪条边）
    std::vector<std::vector<const Edge*>> paths;  //路径的数组，每个路径是一组边的指针，可以存储一个智能体的好多条路径，，（不包括时间）

    Train(uint32_t id, uint32_t length, uint32_t depTime, const Edge* const start);//构造函数，用于初始化火车的属性和起始停靠点

    void addStop(uint32_t arrivalTime, const Edge* const stopEdge);//添加停靠点的函数，用于将停靠点的到达时间和停靠边添加到停靠点列表中。

    bool operator==(const Train& otherTrain) const;//重载等于运算符，用于比较两个火车是否相等。
};


#endif //SATCBS_TRAIN_TRAIN_H
