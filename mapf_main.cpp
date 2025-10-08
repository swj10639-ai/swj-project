#include "mapf_main.h"
#include "z3++.h"
#include <vector>
#include "graph.h"
#include "edge.h"
#include "train.h"
#include "instance.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <set>
#include <stack>
#include <unistd.h> 




using namespace z3;

void print_memory_usage() {
    std::ifstream statm("/proc/self/statm");
    if (!statm) {
        std::cerr << "Failed to open /proc/self/statm" << std::endl;
        return;
    }

    long size, resident, share, text, lib, data, dt;
    statm >> size >> resident >> share >> text >> lib >> data >> dt;

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
    long physical_memory_used_kb = resident * page_size_kb;

    long physical_memory_used_kb_to_kb = physical_memory_used_kb * 8;

    std::cout << "Physical Memory Used:" << std::endl;
    std::cout << "  In Kb: " << physical_memory_used_kb_to_kb << " Kb" << std::endl;

    statm.close();
}

Instance* parseInstance(const std::string& pathToFile) {
    std::ifstream file(pathToFile);
    if(! file.is_open()) {
        std::cerr << "Instance-file not found" << std::endl;
        file.close();
        std::exit(-1);
    }

    int numNodes, maxTimeSteps;
    file >> numNodes >> maxTimeSteps;
    Graph g(numNodes);

    std::string s;
    getline(file, s);
    getline(file, s);
    if(!s.empty()) {
        std::cerr << "Ill-formed instance file" << std::endl;
        file.close();
        std::exit(-1);
    }

    while(getline(file, s) && !s.empty()) {
        int from, to, length;
        std::stringstream tmp(s);
        tmp >> from >> to >> length;
        g.addEdge(from, to, length);
    }

    while(getline(file, s) && !s.empty()) {
        int left, mid, right;
        std::stringstream tmp(s);
        tmp >> left >> mid >> right;
        g.prohibitTransition(left, mid, right);
    }

    std::vector<Train> trains;
    int trainId = -1;

    while(getline(file, s)) {
        int length, depTime, from, to;
        std::stringstream tmp(s);
        tmp >> length >> depTime >> from >> to;
        Train train(++trainId, length, depTime, g.getEdge(from, to));
        while(getline(file, s) && !s.empty()) {
            int arrivalTime, from, to;
            std::stringstream tmp2(s);
            tmp2 >> arrivalTime >> from >> to;
            train.addStop(arrivalTime, g.getEdge(from, to));
        }
        trains.push_back(train);
        if(file.eof()) break;
    }
    file.close();

    return new Instance(g, trains, maxTimeSteps,new context);//执行
}







int main(int argc, char *argv[]) {
    std::string pathToFile = "/home/swj/桌面/SAT_TR/runningExample.txt";
    //std::string pathToFile = "/home/swj/桌面/SAT_TR2/aaa.txt";
    Instance *instance=parseInstance(pathToFile);
    print_memory_usage();




    return 0;






}
