// lidar 2 lidar calibration
#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <algorithm>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/features/normal_3d_omp.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/common/io.h>


typedef pcl::PointNormal PointXYZN;
typedef pcl::PointCloud<PointXYZN> PointCloudXYZN;
typedef PointCloudXYZN::Ptr PointCloudXYZNPtr;

typedef pcl::PointXYZINormal PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

void icp_match(const PointCloudPtr& src, const PointCloudPtr& base, Eigen::Matrix4d &trans) {
  pcl::NormalEstimationOMP<PointT, PointT> norm_est;
  pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());

  norm_est.setNumberOfThreads(8);
  norm_est.setSearchMethod(kdtree);
  norm_est.setKSearch(20);

  norm_est.setInputCloud(src);
  norm_est.compute(*src);
  norm_est.setInputCloud(base);
  norm_est.compute(*base);

  pcl::IterativeClosestPointWithNormals<PointT, PointT, float> icp;
//  pcl::IterativeClosestPoint<PointT, PointT, float> icp;
  icp.setInputSource(src);
  icp.setInputTarget(base);

  std::cout<<"source points size "<<src->size()<<std::endl;
  std::cout<<"target points size "<<base->size()<<std::endl;

  icp.setMaxCorrespondenceDistance(5);       //5
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);       //1e-6
  icp.setEuclideanFitnessEpsilon(1e-3);

  PointCloudPtr src_registered(new PointCloud);
  Eigen::Matrix4f init_pose = trans.cast<float>();

//  modified
//  for de005
//  init_pose << -0.7313537,  0.6819984,  0.0000000,  0,   -0.6819984, -0.7313537,  0.0000000,  0,
//    0.0000000,  0.0000000,  1.0000000,  0,     0,0,0,1;
  for mid360_leica_room1
  init_pose << 0.1953514,  0.3897651, -0.8999561,  0,   -0.7252393,  0.6751375,  0.1349716,  0,     0.6602014,  0.6263166,  0.4145620,  0,     0,0,0,1;
//  for mid360_azure calib
  // init_pose << 0, 0,  1.0000000,  0,   -1,  0,  0.0000000,  0,
  //   0.0000000,  -1.0000000,  0.0000000,  0,     0,0,0,1;

  icp.align(*src_registered, init_pose);

  std::cout<<"align icp final"<<std::endl;

  if (icp.hasConverged()) {
    trans = icp.getFinalTransformation().cast<double>();
//    modified
    std::cout << "mean square error from getFitnessScore is: " << icp.getFitnessScore () << std::endl;
  } else {

    std::cout << "align failed" << std::endl;
  }
}

//modified

// 用于将参数传递给回调函数的结构体
struct CallbackArgs {
    PointCloudPtr clicked_points_3d;
    pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};
void pickPointCallback(const pcl::visualization::PointPickingEvent &event, void *args) {
    CallbackArgs *data = (CallbackArgs *) args;
    if (event.getPointIndex() == -1)
        return;
    PointT current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    data->clicked_points_3d->points.push_back(current_point);
    // 绘制红色点
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(data->clicked_points_3d, 255, 0, 0);
    data->viewerPtr->removePointCloud("clicked_points");
    data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
    data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10,
                                                      "clicked_points");
    std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

void lidar2lidarCalibration(const std::string& path1, const std::string& path2)
{
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr source_pts(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_pts(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::io::loadPCDFile(path1, *source_pts);
  pcl::io::loadPCDFile(path2, *target_pts);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*source_pts, *source_pts, indices);
  pcl::removeNaNFromPointCloud<pcl::PointXYZINormal>(*target_pts, *target_pts, indices);

  pcl::VoxelGrid<pcl::PointXYZINormal> filter;
  filter.setLeafSize(0.02f, 0.02f, 0.02f);
  filter.setInputCloud(source_pts);
  filter.filter(*source_pts);
  filter.setInputCloud(target_pts);
  filter.filter(*target_pts);
  // pcl::io::savePCDFileASCII ("/home/liu/tools/calibration_kit/test_data/lidar2lidar/filtered_points2.pcd", *source_pts);
  // pcl::io::savePCDFileASCII ("/home/liu/tools/calibration_kit/test_data/lidar2lidar/filtered_livox2.pcd", *target_pts);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;

  sor.setInputCloud(source_pts);
  sor.setMeanK(10);
  sor.setStddevMulThresh(1.0);
  sor.filter(*source_pts);

  sor.setInputCloud(target_pts);
  sor.filter(*target_pts);

  std::cout<<"load pcd"<<std::endl;

  Eigen::Matrix4d tran_mat_icp;
  Eigen::Matrix4d tran_mat_teaser;

  tran_mat_icp.setIdentity();

  icp_match(source_pts, target_pts, tran_mat_icp);

  std::cout<<"the calibration result is "<<std::endl<<tran_mat_icp<<std::endl;

// 输出成txt
  // open a file for outputting the matrix
    std::ofstream file("/home/liu/tools/calibration_kit/test_data/lidar2lidar/matrix.txt");

    if (file.is_open()) {
        file << tran_mat_icp; // 将矩阵写入文件
        file.close(); // 关闭文件
        std::cout << "矩阵已保存为 matrix.txt" << std::endl;
    } else {
        std::cout << "无法打开文件" << std::endl;
    }
 
  PointCloudPtr transed_cloud(new PointCloud);
  pcl::transformPointCloud(*source_pts, *transed_cloud, tran_mat_icp);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("trans_viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<PointT> source_color_handle(target_pts, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> target_color_handle(transed_cloud, 0, 255, 0);

  // 合并保存点云
  // pcl::PointCloud<pcl::PointXYZINormal> contrast_pts;
  // contrast_pts = *target_pts;
  // contrast_pts += *transed_cloud;
  // pcl::io::savePCDFileASCII ("/home/liu/tools/calibration_kit/contrast.pcd", contrast_pts);
  // std::cout << "contrast_pts saved." << std::endl;

  viewer->addPointCloud(target_pts, source_color_handle, "source");
  viewer->addPointCloud(transed_cloud, target_color_handle, "target");

//  modified
    CallbackArgs  cb_args;
    PointCloudPtr clicked_points_3d(new PointCloud);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(viewer);
    viewer->registerPointPickingCallback(pickPointCallback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;


  viewer->spin();
  //https://github.com/PointCloudLibrary/pcl/issues/172
  //viewer windows can't close
  viewer->close();
}
