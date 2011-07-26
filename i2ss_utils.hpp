#ifndef I2SS_UTILS_H_
#define I2SS_UTILS_H_

using namespace std;
using namespace pcl;

//There are a ton of parameters that need passing around, so might as well struct them up 
struct base_fields {
  base_fields (int met, int mod, string mtype, string modtype) :
	method(met), model(mod), method_type(mtype), model_type(modtype) {}
  int method;
  int model;
  string method_type;
  string model_type;
};
  
struct params {
  params (int k, float s, float ct, int cm, int cx, double r, double ndw) :
	mean_k(k), stdev(s), ctol(ct), cmin(cm), cmax(cx), sr(r), ndist_w(ndw) {}
  int mean_k;
  float stdev;
  float ctol;
  int cmin;
  int cmax;
  double sr;
  double ndist_w;
};

void deleteOldResults(const string& category) {

  string delete_old_results;

  #ifdef __WIN32
    delete_old_results = "del " + category + "_*.pcd";
  #else
    delete_old_results = "rm " + category + "_*.pcd";
  #endif

  system(delete_old_results.c_str());

}

size_t subtract_segments(base_fields& h, params& p, PointCloud<PointXYZ>::Ptr& input_cloud) {

  ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
  PointIndices::Ptr inliers (new PointIndices ());
  PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ> ()), seg_outliers (new PointCloud<PointXYZ>());
  PCDWriter writer;

  // Create the segmentation object
  SACSegmentation<PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (h.model);
  seg.setMethodType (h.method);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (p.sr);

  // Create the filtering object
  ExtractIndices<PointXYZ> extract;
  StatisticalOutlierRemoval<PointXYZ> orm;
  
  size_t ground_plane_size = 0, total_segment_size = 0;
  int i = 0, nr_points = input_cloud->points.size ();
  // While 10% of the original cloud is still there
  while (input_cloud->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (input_cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      cerr << "Could not estimate the specified model for the given dataset." << endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    orm.setInputCloud(cloud_p);
    orm.setMeanK(p.mean_k);
    orm.setStddevMulThresh(p.stdev);
    orm.filter(*cloud_p); 
    
    size_t segment_size = cloud_p->width * cloud_p-> height;
    std::cerr << "PointCloud representing the planar component: " << segment_size << " data points " 
              << "after SOR." << std::endl;
    
    //SAC methods usually find the largest plane first, which in our case is the ground plane.
    if (i == 0) ground_plane_size = segment_size;
    total_segment_size += segment_size;

    stringstream ss;
    ss << h.method_type << "_result_segment" << i << ".pcd";
    writer.write<PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*input_cloud);

    
	/*//Re-add the SOR'd points to the main cloud (so that the clustering pass can process them)
	orm.setNegative(true);
	orm.filter(*cloud_p);
    *seg_outliers += *cloud_p;

    orm.setNegative(false);
	orm.filter(*cloud_p);
    */
    i++;
  }

  /*
  cerr << "Reinserting " << seg_outliers->size() << " SOR'd points into the cloud for reprocessing.\n";
  *input_cloud += *seg_outliers;
  */

  size_t feature_size = total_segment_size - ground_plane_size;
  cerr << "Segmented out " << feature_size << " points (" << 100.0 * feature_size / nr_points << "% of input cloud)" << endl; 
  return feature_size;

}

size_t subtract_segments_normal(base_fields& h, params& p, PointCloud<PointXYZ>::Ptr& input_cloud) {

  ModelCoefficients::Ptr coefficients (new ModelCoefficients ());
  PointIndices::Ptr inliers (new PointIndices ());
  PointCloud<PointXYZ>::Ptr cloud_p (new PointCloud<PointXYZ> ());
  PCDWriter writer;

  // Create the segmentation object
  SACSegmentationFromNormals<PointXYZ, Normal> seg;

  // Estimate point normals for the entire cloud
  KdTreeFLANN<PointXYZ>::Ptr stree (new KdTreeFLANN<PointXYZ> ());
  NormalEstimation<PointXYZ, Normal> ne;
  PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
  
  ne.setViewPoint(0.0, 0.0, 1.5);
  ne.setSearchMethod (stree);
  ne.setInputCloud (input_cloud);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (h.model);
  seg.setMethodType (h.method);
  seg.setNormalDistanceWeight (p.ndist_w);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (p.sr);

  // Create the filtering objects
  ExtractIndices<PointXYZ> extract;
  ExtractIndices<Normal> extract_normals;
  //StatisticalOutlierRemoval<PointXYZ> orm;

  size_t ground_plane_size = 0, total_segment_size = 0;
  int i = 0, nr_points = input_cloud->points.size ();
  // While 10% of the original cloud is still there
  while (input_cloud->points.size () > 0.1 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (input_cloud);
	seg.setInputNormals(cloud_normals);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      cerr << "Could not estimate the specified model for the given dataset." << endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (input_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);

    /*
    orm.setInputCloud(cloud_p);
    orm.setMeanK(p.mean_k);
    orm.setStddevMulThresh(p.stdev);
    orm.filter(*cloud_p); 
    */
    size_t segment_size = cloud_p->width * cloud_p-> height;
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

	//SAC methods usually find the largest plane first, which in our case is the ground plane.
    if (i == 0) ground_plane_size = segment_size;
    total_segment_size += segment_size;

    stringstream ss;
    ss << h.method_type << "_result_normal_segment" << i << ".pcd";
    writer.write<PointXYZ> (ss.str (), *cloud_p, false);

    // Extract the segmented inliers and their corresponding normals from the original cloud
    extract.setNegative (true);
    extract.filter (*input_cloud);

    extract_normals.setInputCloud(cloud_normals);
	extract_normals.setIndices(inliers);
	extract_normals.setNegative(true);
	extract_normals.filter(*cloud_normals);

    i++;
  }

  size_t feature_size = total_segment_size - ground_plane_size;
  cerr << "Segmented out " << feature_size << " points (" << 100.0 * feature_size / nr_points << "% of input cloud)" << endl; 
  return feature_size;

}

#endif
