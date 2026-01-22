#include <sstream>
#include "edge.h"

Edge::Edge(int id, int from, int to, int length) // 构造函数，用于初始化边的属性和邻接边列表
        : id(id), from(from), to(to), length(length),
          neighbours() {}


int Edge::getOtherVertex(int vertex) const {// 根据给定的顶点，返回边上除该顶点外的另一个顶点
    return vertex == from ? to : from;
}
int Edge::operator[](int vertex) const {
    return getOtherVertex(vertex); } // 重载运算符[]，根据给定的顶点，返回边上除该顶点外的另一个顶点

Edge *Edge::addNeighbour(Edge *neighbour) { // 将相邻的边添加到邻接边列表中，并返回添加的边的指针
    neighbours.push_back(neighbour);
    return neighbours.back(); //neighbours.back();指向neighbours向量中的最后一个元素，即添加的边的指针
}

bool Edge::connectsTo(int vertex) const { // 检查该边是否与给定顶点相连，如果边的目标顶点或起始顶点与给定顶点相等，则返回true，否则返回false
    return vertex == to || vertex == from;
}

int Edge::sharedVertex(const Edge *otherEdge) const {
    if (otherEdge->connectsTo(from))// 获取与另一条边共享的顶点，如果另一条边与当前边连接的顶点与当前边的起始顶点相等，则返回起始顶点
        return from;
    if (otherEdge->connectsTo(to))// 如果另一条边与当前边连接的顶点与当前边的目标顶点相等，则返回目标顶点
        return to;
    return -1;// 如果与另一条边没有共享顶点，则返回-1
}


bool Edge::containsNeighbour(const Edge* otherEdge) const {
    for (const Edge* neighbour : neighbours) {
        if (neighbour == otherEdge) {
            return true;
        }
    }
    return false;
}


std::string Edge::toString() const {    // 将边的起始顶点和目标顶点转换为字符串表示，并返回该字符串
    std::stringstream str;
    str << "from " << from << " to " << to;
    return str.str();
}


