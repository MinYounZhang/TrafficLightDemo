#include <stdio.h>
#include "RosNode.h"

int main(int argc, char **argv)
{
    std::shared_ptr<ParseArgs> args = std::make_shared<ParseArgs>(argc, argv);
    argc = 1;
    ros::init(argc, argv, "TrafficLight");

    // 启动节点
    RosNode node;
    node.Run(args);

    return 0;
}
