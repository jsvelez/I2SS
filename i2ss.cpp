#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "i2ss_utils.hpp"

using namespace std;
using namespace pcl;

int
 main (int argc, char** argv)
{

  if (argc < 3) {
    cerr << "Usage: ./cluster_subtraction input_cloud.pcd distance_threshold [--sor mean_k stdev_threshold] "
         << "[--cluster tolerance min_pts max_pts] [--method ransac | mlesac | rmsac | rransac]\n"
		 << "[--model plane | line | circle | sphere | cylinder | normal_plane | parallel_plane | registration]\n"
		 << "[--normal_weight nwt]\n";
    return -1;
  }

  int opt_index = 3;
  bool arg_error = false;

  //Initializing some defaults...
  base_fields header (SAC_RANSAC, SACMODEL_PLANE, "ransac", "plane");
  params pars (50, 1.0, 0.02, 250, 25000, 0.03, 0.1);

  pars.sr = atof(argv[2]);
  if (pars.sr <= 0.0) {
    cerr << "Using default distance threshold of 0.03.\n";
    pars.sr = 0.03;
  }
  else 
    cerr << "Using distance threshold of " << pars.sr << ".\n";

  //Because this is less messy than a chain of else-ifs later on :)
  map<string, int> models;

  models["plane"] = SACMODEL_PLANE;
  models["line"] = SACMODEL_LINE;
  models["circle"] = SACMODEL_CIRCLE2D;
  models["sphere"] = SACMODEL_SPHERE;
  models["cylinder"] = SACMODEL_CYLINDER;
  models["normal_plane"] = SACMODEL_NORMAL_PLANE;
  models["parallel_plane"] = SACMODEL_PARALLEL_PLANE;
  models["registration"] = SACMODEL_REGISTRATION;

  typedef map<string, int>::const_iterator ModelIterator;
  typedef vector<int>::const_iterator IndexIterator;

  //Yes, this is very messy. It's a miracle it works at all, but hey!
  while ((opt_index < argc) && (strncmp(argv[opt_index], "--", 2) == 0)) {
	string _switch = argv[opt_index];
	if (_switch == "--sor") {
		if (opt_index + 2 >= argc) {
			arg_error = true;
			break;
		}
		pars.mean_k = atof(argv[++opt_index]);
		pars.stdev = atof(argv[++opt_index]);
	}
	else if (_switch == "--cluster") {
		if (opt_index + 3 >= argc) {
			arg_error = true;
			break;
		}
		pars.ctol = atof(argv[++opt_index]);
		pars.cmin = atoi(argv[++opt_index]);
		pars.cmax = atoi(argv[++opt_index]);
	}
	else if (_switch == "--method") {
		if (opt_index + 1 >= argc) {
			arg_error = true;
			break;
		}
		header.method_type = argv[++opt_index];

		cerr << "Method type is: " << header.method_type << "\n";

		if (header.method_type == "ransac") header.method = SAC_RANSAC;
		else if (header.method_type == "mlesac") header.method = SAC_MLESAC;
		else if (header.method_type == "rmsac") header.method = SAC_RMSAC;
		else if (header.method_type == "rransac") header.method = SAC_RRANSAC;
		else cerr << "Unknown method type '" << header.method_type << "'; defaulting to SAC_RANSAC.\n"; 
	}
	else if (_switch == "--model") {
		if (opt_index + 1 >= argc) {
			arg_error = true;
			break;
		}
		header.model_type = argv[++opt_index];
		ModelIterator it;
		it = models.find(header.model_type);
		if (it == models.end()) 
			cerr << "Unknown model type '" << header.model_type << "'; defaulting to SACMODEL_PLANE.\n";
		else header.model = it->second;
	}
	else if (_switch == "--normal_weight") {
		if (opt_index + 1 >= argc) {
			arg_error = true;
			break;
		}
		pars.ndist_w = atof(argv[++opt_index]);
	}
	else {
		cerr << "Usage: ./cluster_subtraction input_cloud.pcd distance_threshold [--sor mean_k stdev_threshold] "
             << "[--cluster tolerance min_pts max_pts]\n"
			 << "[--model plane | line | circle | sphere | cylinder | normal_plane | parallel_plane | registration]\n";
		arg_error = true;
	}
    ++opt_index;
  }

  if (arg_error) return -1;

  cerr << "Mean K for statistical outlier removal is set to " << pars.mean_k << ".\n"
       << "Standard deviation threshold is " << pars.stdev << ".\n\n";

  cerr << "Cluster tolerance set to " << pars.ctol << ".\n"
       << "Minimum number of cluster points set to " << pars.cmin << ".\n"
       << "Maximum number of cluster points set to " << pars.cmax << ".\n";

  sensor_msgs::PointCloud2::Ptr cloud_blob (new sensor_msgs::PointCloud2), cloud_filtered_blob (new sensor_msgs::PointCloud2);
  PointCloud<PointXYZ>::Ptr cloud_filtered (new PointCloud<PointXYZ>); 

  // Fill in the cloud data
  PCDReader reader;
  reader.read (argv[1], *cloud_blob);

  //Delete old results, if any
  deleteOldResults(header.method_type);

  size_t cloud_size = cloud_blob->width * cloud_blob-> height;
  cerr << "PointCloud before filtering: " << cloud_size << " data points." << endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  fromROSMsg (*cloud_filtered_blob, *cloud_filtered);

  size_t cloud_filtered_size = cloud_filtered->width * cloud_filtered->height;

  cerr << "PointCloud after filtering: " << cloud_filtered_size << " data points." 
       << " (" << 100.0*cloud_filtered_size/cloud_size << "% preserved)" << endl;

  // Write the downsampled version to disk
  PCDWriter writer;
  const string fname = argv[1];
  writer.write<PointXYZ> (fname + "_downsampled.pcd", *cloud_filtered, false);

  size_t useful_segment_points = 0;
  if (header.model == SACMODEL_NORMAL_PLANE) 
	useful_segment_points = subtract_segments_normal(header, pars, cloud_filtered);
  else 
	useful_segment_points = subtract_segments(header, pars, cloud_filtered);

  // Creates the KdTree object for the search method of the extraction
  KdTree<PointXYZ>::Ptr tree (new KdTreeFLANN<PointXYZ>);
  tree->setInputCloud (cloud_filtered);
  
  vector<PointIndices> cluster_indices; //a vector of vector<int>s that will contain the indices for the points in each cluster

  //Extracts clusters of points close enough to each other
  EuclideanClusterExtraction<PointXYZ> ec;
  ec.setClusterTolerance (pars.ctol); 
  ec.setMinClusterSize (pars.cmin);
  ec.setMaxClusterSize (pars.cmax);
  ec.setSearchMethod (tree);
  ec.setInputCloud( cloud_filtered);
  ec.extract (cluster_indices);

  //Writes each point cluster into its own cloud object and saves them to disk.
  int j = 0;
  size_t cluster_points = 0;
  for (vector<PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
      PointCloud<PointXYZ>::Ptr cloud_cluster (new PointCloud<PointXYZ>);
      for (IndexIterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*

      cerr << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << endl;
	  cluster_points += cloud_cluster->points.size();
      stringstream ss;
      ss << header.method_type << "_result_cluster" << j << ".pcd";
      writer.write<PointXYZ> (ss.str (), *cloud_cluster, false); //*
      j++;
  }

  cerr << "Clustered out " << cluster_points << " points (" << 100.0 * cluster_points / cloud_filtered_size << "% of input cloud)" << endl;

  size_t total_useful_points = useful_segment_points + cluster_points;
  cerr << "Total amount of points of interest: " << total_useful_points << "/" << cloud_filtered_size 
       << " (" << 100.0 * total_useful_points / cloud_filtered_size << "% of cloud)\n";

  return (0);
}

