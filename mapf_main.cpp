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
#include <unistd.h> // 添加这一行以包含 sysconf




using namespace z3;

void print_memory_usage() {
    std::ifstream statm("/proc/self/statm");
    if (!statm) {
        std::cerr << "Failed to open /proc/self/statm" << std::endl;
        return;
    }

    long size, resident, share, text, lib, data, dt;
    statm >> size >> resident >> share >> text >> lib >> data >> dt;

    long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // 页面大小（以KB为单位）
    long physical_memory_used_kb = resident * page_size_kb; // 实际使用的物理内存（KB）

    // 转换为 Kb (bit)
    long physical_memory_used_kb_to_kb = physical_memory_used_kb * 8; // KB 转 Kb (1 KB = 8 Kb)

    std::cout << "Physical Memory Used:" << std::endl;
    std::cout << "  In Kb: " << physical_memory_used_kb_to_kb << " Kb" << std::endl;

    statm.close();
}

Instance* parseInstance(const std::string& pathToFile) {
    std::ifstream file(pathToFile);     // 打开指定路径的文件
    if(! file.is_open()) {
        std::cerr << "Instance-file not found" << std::endl;    //打开失败，无法找到实例文件
        file.close();
        std::exit(-1);
    }

    int numNodes, maxTimeSteps;
    file >> numNodes >> maxTimeSteps; // 从文件中读取节点数量和最大时间步数
    Graph g(numNodes);    // 创建图对象，用于存储图的节点和边信息。

    std::string s;
    getline(file, s);
    getline(file, s);
    if(!s.empty()) {
        std::cerr << "Ill-formed instance file" << std::endl;   //实例文件格式错误
        file.close();
        std::exit(-1);
    }

    //read edges读取边
    while(getline(file, s) && !s.empty()) {
        int from, to, length;
        std::stringstream tmp(s);
        tmp >> from >> to >> length;
        g.addEdge(from, to, length); // 添加边到图中
    }

    // 读取禁止过渡，，，这里就是禁止道岔了！！！
    while(getline(file, s) && !s.empty()) {
        int left, mid, right;
        std::stringstream tmp(s);
        tmp >> left >> mid >> right;
        g.prohibitTransition(left, mid, right);// 添加禁止过渡到图中
    }

    std::vector<Train> trains;
    int trainId = -1;

    // 读取列车，，，，，，，添加了到达边以及到达边的时间是什么意思
    while(getline(file, s)) {
        int length, depTime, from, to;
        std::stringstream tmp(s);
        tmp >> length >> depTime >> from >> to;
        Train train(++trainId, length, depTime, g.getEdge(from, to));// 创建列车对象
        //read trainstops  读取列车停靠站点
        while(getline(file, s) && !s.empty()) {
            int arrivalTime, from, to;
            std::stringstream tmp2(s);
            tmp2 >> arrivalTime >> from >> to;
            train.addStop(arrivalTime, g.getEdge(from, to)); // 添加停靠站点到列车中
        }
        trains.push_back(train);    // 将列车添加到列车集合中
        if(file.eof()) break;// 创建并返回实例对象
    }
    file.close();

    return new Instance(g, trains, maxTimeSteps,new context);//执行
}//解析地图，构建instance实例







int main(int argc, char *argv[]) {
    std::string pathToFile = "/home/swj/桌面/SAT_TR2/da_etcs1_12.txt";
    //std::string pathToFile = "/home/swj/桌面/SAT_TR/runningExample.txt";
    //std::string pathToFile = "/home/swj/桌面/SAT_TR2/aaa.txt";
    Instance *instance=parseInstance(pathToFile);//构造了一个mapf问题实例
    print_memory_usage(); // 监控内存使用情况




    return 0;





}