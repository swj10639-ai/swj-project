
#ifndef SATCBS_TRAIN_EDGE_H
#define SATCBS_TRAIN_EDGE_H

#include <vector>
#include <string>

class Edge {
    public:
        const int from, to, length ,id;
        std::vector <Edge*> neighbours;

        Edge(int id, int from, int to, int length=1);

        int getOtherVertex(int vertex) const;

        int operator[] (int vertex) const;

        Edge *addNeighbour(Edge *neighbour);
        bool connectsTo(int vertex) const;
        int sharedVertex(const Edge *otherEdge) const;
        bool containsNeighbour(const Edge *otherEdge) const;

        std::string toString() const;

};


#endif //SATCBS_TRAIN_EDGE_H
