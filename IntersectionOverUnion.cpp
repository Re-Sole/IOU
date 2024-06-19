/*
    For this algorithm to work we will need to meet the following hypotesis:
    1) Every label MUST be used only once in the point cloud annotation, only for the same object/part of the scene
    2) The user annotated point cloud HAS to have the same number of used labels as in the ground truth,
        otherwise the IOU resulting will be inaccurate
*/

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#define LABELS_NUM 9

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointXYZRGBNormalCloud;

int main(int argc, char* argv[])
{
    if(argc != 3) {
        std::cout << "Usage: " << argv[0] << " ground_truth_filename user_filename" << std::endl;
        return -1;
    }

    PointXYZRGBNormalCloud gt, user;
    pcl::io::loadPCDFile(std::string(argv[1]), gt);
    pcl::io::loadPCDFile(std::string(argv[2]), user);

    int point_cloud_points_num = gt.size();

    int tot_intersecting_points = 0;
    for(int gt_label = 0; gt_label < LABELS_NUM; gt_label++) {
        int max_intersection = 0;
        for(int user_label = 0; user_label < LABELS_NUM; user_label++) {
            int intersecting_points = 0;
            for(int i = 0; i < point_cloud_points_num; i++) {
                if(gt[i].curvature == gt_label && user[i].curvature == user_label) intersecting_points++;
            }
            // std::cout << "Groud truth label: " << gt_label << ", user label: " << user_label << ", intersecting points: " << intersecting_points << std::endl;
            max_intersection = std::max(max_intersection, intersecting_points);
        }
        tot_intersecting_points += max_intersection;
    }

    float IOU = (float)tot_intersecting_points / (float)point_cloud_points_num;
    std::cout << "IOU: " << IOU << std::endl;

    return 0;
}