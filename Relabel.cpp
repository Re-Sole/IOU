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
    if (argc != 3) {
        std::cout << "Usage: " << std::string(argv[0]) << " INPUT_FILENAME OUTPUT_FILENAME" << std::endl;
        return 1;
    }

    PointCloud pc;
    std::string in_filename = std::string(argv[1]);
    std::string out_filename = std::string(argv[2]);
    pcl::io::loadPCDFile(in_filename, pc);
    std::cout << "Points: " << pc.size() << std::endl;

    // set all the points with label 8 (GRAY) to 0 (NO_LABEL)
    for (int i = 0; i < pc.size(); i++) {
#ifdef USE_CURVATURE
        if (pc[i].curvature == 8) {
            pc[i].curvature = 0;
        }
#else
        if (pc[i].label == 8) {
            pc[i].label = 0;
            // set also the color to WHITE
            pc[i].r = 0xff;
            pc[i].g = 0xff;
            pc[i].b = 0xff;
        }
#endif
    }

    // save output file
    pcl::io::savePCDFileBinary(out_filename, pc);
    std::cout << "Save done" << std::endl;

    return 0;
}