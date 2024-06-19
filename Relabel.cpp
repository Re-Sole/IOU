#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointXYZRGBNormalCloud;

int main(int argc, char* argv[])
{
    std::string input_folder = "../annotazioni_originali/";
    std::string output_folder = "../annotazioni/";

    {
        std::string filename = "expert_annotation.pcd";

        PointXYZRGBNormalCloud pc;
        pcl::io::loadPCDFile(input_folder + filename, pc);

        std::cout << "Points: " << pc.size() << std::endl;

        for (int i = 0; i < pc.size(); i++) {
            if(pc[i].curvature == 1) {
                if(pc[i].x <= 0 && pc[i].z > 0) {
                    pc[i].curvature = 5;
                } else if(pc[i].x > 0 && pc[i].z > 0) {
                    pc[i].curvature = 6;
                } else if(pc[i].x > 0 && pc[i].z < 0) {
                    pc[i].curvature = 7;
                }
            }
        }

        // save output file
        pcl::io::savePCDFileBinary(output_folder + filename, pc);
        std::cout << "Save done" << std::endl;
    }

    {
        std::string filename = "expert_annotation_different_labels.pcd";

        PointXYZRGBNormalCloud pc;
        pcl::io::loadPCDFile(input_folder + filename, pc);

        std::cout << "Points: " << pc.size() << std::endl;

        for (int i = 0; i < pc.size(); i++) {
            if(pc[i].curvature == 1) {
                if(pc[i].x <= 0 && pc[i].z > 0) {
                    pc[i].curvature = 5;
                } else if(pc[i].x > 0 && pc[i].z > 0) {
                    pc[i].curvature = 7;
                } else if(pc[i].x > 0 && pc[i].z < 0) {
                    pc[i].curvature = 8;
                }
            }
        }

        // save output file
        pcl::io::savePCDFileBinary(output_folder + filename, pc);
        std::cout << "Save done" << std::endl;
    }

    {
        std::string filename = "expert_annotation_with_little_difference.pcd";

        PointXYZRGBNormalCloud pc;
        pcl::io::loadPCDFile(input_folder + filename, pc);

        std::cout << "Points: " << pc.size() << std::endl;

        for (int i = 0; i < pc.size(); i++) {
            if(pc[i].curvature == 1) {
                if(pc[i].x <= 0 && pc[i].z > 0) {
                    pc[i].curvature = 5;
                } else if(pc[i].x > 0 && pc[i].z > 0) {
                    pc[i].curvature = 7;
                } else if(pc[i].x > 0 && pc[i].z < 0) {
                    pc[i].curvature = 8;
                }
            }
        }

        // save output file
        pcl::io::savePCDFileBinary(output_folder + filename, pc);
        std::cout << "Save done" << std::endl;
    }

    {
        std::string filename = "expert_annotation_without_1_label.pcd";

        PointXYZRGBNormalCloud pc;
        pcl::io::loadPCDFile(input_folder + filename, pc);

        std::cout << "Points: " << pc.size() << std::endl;

        for (int i = 0; i < pc.size(); i++) {
            if(pc[i].curvature == 1) {
                if(pc[i].x <= 0 && pc[i].z > 0) {
                    pc[i].curvature = 5;
                } else if(pc[i].x > 0 && pc[i].z > 0) {
                    pc[i].curvature = 6;
                } else if(pc[i].x > 0 && pc[i].z < 0) {
                    pc[i].curvature = 7;
                }
            }
        }

        // save output file
        pcl::io::savePCDFileBinary(output_folder + filename, pc);
        std::cout << "Save done" << std::endl;
    }

    return 0;
}