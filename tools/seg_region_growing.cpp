/*
 * =====================================================================================
 *
 *       Filename:  seg_region_growing.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  10/24/2018 03:04:31 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#include <stdlib.h>

#include <iostream>
#include <string>
#include <Eigen/Core>
#include <pcl/common/pca.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace Eigen;
using namespace std;
void printHelp(int, char** argv)

{
  print_error("Syntax is: %s input_xyzinorml.pcd  <options>\n", argv[0]);
  print_info("  where options are:\n");
  // print_value("%f", default_search_radius);
  print_info(")\n");
}

bool loadCloud(const std::string& filename, pcl::PCLPointCloud2& cloud)
{
  TicToc tt;
  print_highlight("Loading ");
  print_value("%s ", filename.c_str());

  tt.tic();
  if (loadPCDFile(filename, cloud) < 0)
    return (false);
  print_info("[done, ");
  print_value("%g", tt.toc());
  print_info(" ms : ");
  print_value("%d", cloud.width * cloud.height);
  print_info(" points]\n");
  print_info("Available dimensions: ");
  print_value("%s\n", pcl::getFieldsList(cloud).c_str());
  return (true);
}
int main(int argc, char** argv)
{
  print_info("区域增长分割. For more information, use: %s -h\n", argv[0]);
  if (argc < 2)
  {
    printHelp(argc, argv);
    return (-1);
  }
  // Load the first file
  pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
  if (!loadCloud(argv[1], *cloud2))
  {
    return (-1);
  }
  pcl::PointCloud<pcl::PointXYZINormal> pcloud;

  pcl::fromPCLPointCloud2(*cloud2, pcloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_n(new pcl::PointCloud<pcl::Normal>);
  for (std::size_t i = 0, k = 0; i < pcloud.size(); ++i)
  {
    pcl::PointXYZ p;
    p.x = pcloud.at(i).x;
    p.y = pcloud.at(i).y;
    p.z = pcloud.at(i).z;
    cloud_xyz->push_back(p);
    pcl::Normal p_n;
    p_n.normal_x = pcloud.at(i).normal_x;
    p_n.normal_y = pcloud.at(i).normal_y;
    p_n.normal_z = pcloud.at(i).normal_z;

    if (p_n.normal_z < 0)
    {
      p_n.normal_z *= -1;
    }
    cloud_n->push_back(p_n);
  }
  pcl::io::savePCDFileASCII(argv[2], *cloud_n);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree =
      boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
  std::cout << "point size is " << pcloud.size() << std::endl;
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud_xyz);
  // reg.setIndices (indices);
  reg.setInputNormals(cloud_n);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
  std::cout << "These are the indices of the points of the initial" << std::endl
            << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  while (counter < clusters.size())
  {
    std::cout << clusters[counter].indices.size() << ", ";
    counter++;
  }
  std::cout << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  pcl::visualization::CloudViewer viewer("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped())
  {
  }

  return (0);
}
