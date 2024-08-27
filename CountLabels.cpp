#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

//#define USE_CURVATURE // comment to use label

#ifdef USE_CURVATURE
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloud;
#else
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud;
#endif


int main(int argc, char* argv[])
{
    const int LABELS_NUM = 9;

    if(argc != 2) {
        std::cout << "Usage: " << argv[0] << " PCD_FILE" << std::endl;
        return -1;
    }

    PointCloud pc;
    pcl::io::loadPCDFile(std::string(argv[1]), pc);
    std::cout << "Points: " << pc.size() << std::endl;

    int labels_count[LABELS_NUM] = {0};
    for (int i = 0; i < pc.size(); i++) {
#ifdef USE_CURVATURE
        int label = pc[i].curvature;
#else
        int label = pc[i].label;
#endif
        labels_count[label]++;
    }

    for (int i = 0; i < LABELS_NUM; i++) {
        std::cout << "Label " << i << ": " << labels_count[i] << " points" << std::endl;
    }

    return 0;
}