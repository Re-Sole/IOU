#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define LABELS_NUM 9

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointXYZRGBNormalCloud;

int main(int argc, char* argv[])
{
    if(argc != 2) {
        std::cout << "Usage: " << argv[0] << " FILE1" << std::endl;
        return -1;
    }

    PointXYZRGBNormalCloud pc;
    pcl::io::loadPCDFile(std::string(argv[1]), pc);

    std::cout << "Points: " << pc.size() << std::endl;

    int labels_count[LABELS_NUM] = {0};

    for (int i = 0; i < pc.size(); i++) {
        int label = pc[i].curvature;
        labels_count[label]++;
    }

    for (int i = 0; i < LABELS_NUM; i++) {
        std::cout << "Label " << i << ": " << labels_count[i] << " points" << std::endl;
    }

    return 0;
}