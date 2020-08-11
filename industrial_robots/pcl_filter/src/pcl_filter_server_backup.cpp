#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <boost/foreach.hpp>
#include <pcl/filters/conditional_removal.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "pcl_filter/color_filter.h"
#include "pcl_filter/shape_filter.h"

#include <string>

#define RED 1
#define GREEN 2
#define BLUE 3
#define YELLOW 4

#define SPHERE 1
#define CYLINDER 2

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointXYZ PointT;

struct ColorRange
{
  int rMax;
  int rMin;
  int bMax;
  int bMin;
  int gMax;
  int gMin;
};

class ObjectDetection
{
public:
  ObjectDetection()
  {
    pub_green_sphere = nh.advertise<PointCloud> ("/green_sphere", 1);
    pub_red_sphere = nh.advertise<PointCloud> ("/red_sphere", 1);
    pub_blue_sphere = nh.advertise<PointCloud> ("/blue_sphere", 1);
    pub_yellow_sphere = nh.advertise<PointCloud> ("/yellow_sphere", 1);
    pub_green_cylinder = nh.advertise<PointCloud> ("/green_cylinder", 1);
    pub_red_cylinder = nh.advertise<PointCloud> ("/red_cylinder", 1);
    pub_blue_cylinder = nh.advertise<PointCloud> ("/blue_cylinder", 1);
    pub_yellow_cylinder = nh.advertise<PointCloud> ("/yellow_cylinder", 1);

    pub_blue = nh.advertise<PointCloudRGB> ("/blue_filter", 1);
    pub_red = nh.advertise<PointCloudRGB> ("/red_filter", 1);
    pub_green = nh.advertise<PointCloudRGB> ("/green_filter", 1);
    pub_yellow = nh.advertise<PointCloudRGB> ("/yellow_filter", 1);

    image_pub_blue = nh.advertise<sensor_msgs::Image>("blue_filtered_image",1);
    image_pub_green = nh.advertise<sensor_msgs::Image>("green_filtered_image",1);
    image_pub_red = nh.advertise<sensor_msgs::Image>("red_filtered_image",1);
    image_pub_yellow = nh.advertise<sensor_msgs::Image>("yellow_filtered_image",1);

    image_pub_green_sphere = nh.advertise<sensor_msgs::Image> ("/green_sphere_image", 1);
    image_pub_red_sphere = nh.advertise<sensor_msgs::Image> ("/red_sphere_image", 1);
    image_pub_blue_sphere = nh.advertise<sensor_msgs::Image> ("/blue_sphere_image", 1);
    image_pub_yellow_sphere = nh.advertise<sensor_msgs::Image> ("/yellow_sphere_image", 1);
    image_pub_green_cylinder = nh.advertise<sensor_msgs::Image> ("/green_cylinder_image", 1);
    image_pub_red_cylinder = nh.advertise<sensor_msgs::Image> ("/red_cylinder_image", 1);
    image_pub_blue_cylinder = nh.advertise<sensor_msgs::Image> ("/blue_cylinder_image", 1);
    image_pub_yellow_cylinder = nh.advertise<sensor_msgs::Image> ("/yellow_cylinder_image", 1);

    service_color_filter = nh.advertiseService<pcl_filter::color_filter> ("color_filter_server", &ObjectDetection::call_color_filter);
    service_shape_filter = nh.advertiseService<pcl_filter::shape_filter> ("shape_filter_server", &ObjectDetection::call_shape_filter);
  }

  void call_color_filter(const std::shared_ptr<pcl_filter::color_filter::Request> req,
                        std::shared_ptr<pcl_filter::color_filter::Response> res)
  {
    if(req->status == true){
      // start color filter
      switch(req->color){
        case RED:
          assign_color_range(red_range, req->rmax, req->rmin, req->gmax, req->gmin, req->bmax, req->bmin);
          sub_red = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::redfilter_callback, this);
          break;
        case GREEN:
          assign_color_range(green_range, req->rmax, req->rmin, req->gmax, req->gmin, req->bmax, req->bmin);
          sub_green = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::greenfilter_callback, this);
          break;
        case BLUE:
          assign_color_range(blue_range, req->rmax, req->rmin, req->gmax, req->gmin, req->bmax, req->bmin);
          sub_blue = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::bluefilter_callback, this);
          break;
        case YELLOW:
          assign_color_range(blue_range, req->rmax, req->rmin, req->gmax, req->gmin, req->bmax, req->bmin);
          sub_yellow = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::yellowfilter_callback, this);
          break;
      }
      res->status = true;
    }
    else{
      // stop color filter
      switch(req->color){
        case BLUE:
          sub_blue.shutdown();
        case RED:
          sub_red.shutdown();
        case GREEN:
          sub_green.shutdown();
        case YELLOW:
          sub_yellow.shutdown();
      }
      res->status = false;
    }
  }

  void call_shape_filter(const std::shared_ptr<pcl_filter::shape_filter::Request> req,
                        std::shared_ptr<pcl_filter::shape_filter::Response> res)
  {
    if(req->status == true){
      if(req->color == RED && req->shape == SPHERE)
      {
        red_sphere_radius = req->radius;
        sub_red_sphere = nh.subscribe<PointCloud>("/red_filter", 1, &ObjectDetection::red_sphere_callback, this);
      }
      else if(req->color == RED && req->shape == CYLINDER)
      {
        red_cylinder_radius = req->radius;
        sub_red_cylinder = nh.subscribe<PointCloud>("/red_filter", 1, &ObjectDetection::red_cylinder_callback, this);
      }
      else if(req->color == GREEN && req->shape == SPHERE)
      {
        green_sphere_radius = req->radius;
        sub_green_sphere = nh.subscribe<PointCloud>("/green_filter", 1, &ObjectDetection::green_sphere_callback, this);
      }
      else if(req->color == GREEN && req->shape == CYLINDER)
      {
        green_cylinder_radius = req->radius;
        sub_green_cylinder = nh.subscribe<PointCloud>("/green_filter", 1, &ObjectDetection::green_cylinder_callback, this);
      }
      else if(req->color == BLUE && req->shape == SPHERE)
      {
        blue_sphere_radius = req->radius;
        sub_blue_sphere = nh.subscribe<PointCloud>("/blue_filter", 1, &ObjectDetection::blue_sphere_callback, this);
      }
      else if(req->color == BLUE && req->shape == CYLINDER)
      {
        blue_cylinder_radius = req->radius;
        sub_blue_cylinder = nh.subscribe<PointCloud>("/blue_filter", 1, &ObjectDetection::blue_cylinder_callback, this);
      }
      else if(req->color == YELLOW && req->shape == SPHERE)
      {
        yellow_sphere_radius = req->radius;
        sub_yellow_sphere = nh.subscribe<PointCloud>("/yellow_filter", 1, &ObjectDetection::yellow_sphere_callback, this);
      }
      else if(req->color == YELLOW && req->shape == CYLINDER)
      {
        yellow_cylinder_radius = req->radius;
        sub_yellow_cylinder = nh.subscribe<PointCloud>("/yellow_filter", 1, &ObjectDetection::yellow_cylinder_callback, this);
      }

      res->status = true;
    }
    else{
      if(req->color == RED && req->shape == SPHERE)
        sub_red_sphere.shutdown();
      else if(req->color == RED && req->shape == CYLINDER)
        sub_red_cylinder.shutdown();
      else if(req->color == GREEN && req->shape == SPHERE)
        sub_green_sphere.shutdown();
      else if(req->color == GREEN && req->shape == CYLINDER)
        sub_green_cylinder.shutdown();
      else if(req->color == BLUE && req->shape == SPHERE)
        sub_blue_sphere.shutdown();
      else if(req->color == BLUE && req->shape == CYLINDER)
        sub_blue_cylinder.shutdown();
      else if(req->color == YELLOW && req->shape == SPHERE)
        sub_yellow_sphere.shutdown();
      else if(req->color == YELLOW && req->shape == CYLINDER)
        sub_yellow_cylinder.shutdown();

      res->status = false;
    }
    
    // return true;
  }

  void assign_color_range(ColorRange &color_range, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin){
    color_range.rMax = rMax;
    color_range.rMin = rMin;
    color_range.gMax = gMax;
    color_range.gMin = gMin;
    color_range.bMax = bMax;
    color_range.bMin = bMin;
  }

  void yellowfilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, yellow_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, yellow_range.bMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, yellow_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, yellow_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, yellow_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, yellow_range.gMin)));

    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_yellow.publish(cloud_color_filtered);
    // pointcloud_to_rgb_image(cloud_color_filtered, image_pub_yellow);
  }

  void bluefilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, blue_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, blue_range.bMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, blue_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, blue_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, blue_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, blue_range.gMin)));


    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_blue.publish(cloud_color_filtered);
    // pointcloud_to_rgb_image(cloud_color_filtered, image_pub_blue);
  }

  void redfilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, red_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, red_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, red_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, red_range.gMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, red_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, red_range.bMin)));

    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_red.publish(cloud_color_filtered);
    // pointcloud_to_rgb_image(cloud_color_filtered, image_pub_red);
  }

  void greenfilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, green_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, green_range.gMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, green_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, green_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, green_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, green_range.bMin)));

    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_green.publish(cloud_color_filtered);
    // pointcloud_to_rgb_image(cloud_color_filtered, image_pub_green);
  }

  void green_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    detect_sphere(msg, pub_green_sphere, image_pub_green_sphere, "green_sphere", green_sphere_radius);
  }

  void red_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    detect_sphere(msg, pub_red_sphere, image_pub_red_sphere, "red_sphere", red_sphere_radius);
  }

  void blue_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    detect_sphere(msg, pub_blue_sphere, image_pub_blue_sphere, "blue_sphere", blue_sphere_radius);
  }

  void yellow_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    detect_sphere(msg, pub_yellow_sphere, image_pub_yellow_sphere, "yellow_sphere", yellow_sphere_radius);
  }

  void green_cylinder_callback(const PointCloud::ConstPtr& msg)
  {
    detect_cylinder(msg, pub_green_cylinder, image_pub_green_cylinder, "green_cylinder", green_cylinder_radius);
  }

  void red_cylinder_callback(const PointCloud::ConstPtr& msg)
  {
    detect_cylinder(msg, pub_red_cylinder, image_pub_red_cylinder, "red_cylinder", red_cylinder_radius);
  }

  void blue_cylinder_callback(const PointCloud::ConstPtr& msg)
  {
    detect_cylinder(msg, pub_blue_cylinder, image_pub_blue_cylinder, "blue_cylinder", blue_cylinder_radius);
  }

  void yellow_cylinder_callback(const PointCloud::ConstPtr& msg)
  {
    detect_cylinder(msg, pub_yellow_cylinder, image_pub_yellow_cylinder, "yellow_cylinder", yellow_cylinder_radius);
  }

  void detect_cylinder(const PointCloud::ConstPtr& cloud, ros::Publisher pub, ros::Publisher image_pub, std::string frame_id, float radius)
  {
    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    PointCloud::Ptr cloud_filtered2 (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1);
    pass.filter (*cloud_filtered);
    // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    if(cloud_filtered->points.size ()<1000)
      return;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    if(cloud_filtered2->points.size ()<1000)
      return;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, radius);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the sphere inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    // std::cerr << "cylinder coefficients: " << *coefficients_cylinder << std::endl;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(coefficients_cylinder->values[0], coefficients_cylinder->values[1], coefficients_cylinder->values[2]) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), coefficients_cylinder->header.frame_id, frame_id));

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);

    pub.publish(cloud_cylinder);
    pointcloud_to_depth_image(cloud_cylinder, image_pub);
  }

  void detect_sphere(const PointCloud::ConstPtr& cloud, ros::Publisher pub, ros::Publisher image_pub, std::string frame_id, float radius)
  {
    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Datasets
    PointCloud::Ptr cloud_filtered (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    PointCloud::Ptr cloud_filtered2 (new PointCloud);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1);
    pass.filter (*cloud_filtered);
    // std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    if(cloud_filtered->points.size ()<1000)
      return;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    if(cloud_filtered2->points.size ()<1000)
      return;

    // Create the segmentation object for sphere segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, radius);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the sphere inliers and coefficients
    seg.segment (*inliers_sphere, *coefficients_sphere);
    // std::cerr << "sphere coefficients: " << *coefficients_sphere << std::endl;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //float radius = coefficients_sphere->values[3];
    transform.setOrigin( tf::Vector3(coefficients_sphere->values[0], coefficients_sphere->values[1], coefficients_sphere->values[2]) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), coefficients_sphere->header.frame_id, frame_id));

    // Write the sphere inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_sphere);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_sphere);

    pub.publish(cloud_sphere);
    pointcloud_to_depth_image(cloud_sphere, image_pub);
  }

  void pointcloud_to_depth_image(const PointCloud::ConstPtr& msg, ros::Publisher pub)
  {
    float centre_x = 320.5;
    float centre_y = 240.5;
    float focal_x = 554.254691191187;
    float focal_y = 554.254691191187;
    int height = 480;
    int width = 640;

    cv::Mat cv_image = cv::Mat(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

    for (int i=0; i<msg->points.size();i++){
      if (msg->points[i].z == msg->points[i].z){
        float z = msg->points[i].z*1000.0;
        float u = (msg->points[i].x*1000.0*focal_x) / z;
        float v = (msg->points[i].y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

        if (pixel_pos_x > (width-1)){
          pixel_pos_x = width -1;
        }
        if (pixel_pos_y > (height-1)){
          pixel_pos_y = height-1;
        }
        cv_image.at<float>(pixel_pos_y,pixel_pos_x) = z;
      }       
    }

    cv_image.convertTo(cv_image,CV_8UC1);

    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "16UC1", cv_image).toImageMsg();
    output_image->header.frame_id = "camera_depth_optical_frame";
    // output_image->header.stamp = info.header.stamp = t;
    pub.publish(output_image);
  }

  void pointcloud_to_rgb_image(const PointCloudRGB::ConstPtr& msg, ros::Publisher pub)
  {
    float centre_x = 320.5;
    float centre_y = 240.5;
    float focal_x = 554.254691191187;
    float focal_y = 554.254691191187;
    int height = 480;
    int width = 640;

    cv::Mat cv_image = cv::Mat(height, width, CV_8UC3);

    for (int i=0; i<msg->points.size();i++){
      if (msg->points[i].z == msg->points[i].z){
        float z = msg->points[i].z*1000.0;
        float u = (msg->points[i].x*1000.0*focal_x) / z;
        float v = (msg->points[i].y*1000.0*focal_y) / z;
        int pixel_pos_x = (int)(u + centre_x);
        int pixel_pos_y = (int)(v + centre_y);

        int r = msg->points[i].r;
        int g = msg->points[i].g;
        int b = msg->points[i].b;

        if (pixel_pos_x > (width-1)){
          pixel_pos_x = width -1;
        }
        if (pixel_pos_y > (height-1)){
          pixel_pos_y = height-1;
        }

        cv_image.at<cv::Vec3b>(pixel_pos_y,pixel_pos_x) = cv::Vec3b(b, g, r);
      }       
    }

    sensor_msgs::ImagePtr output_image = cv_bridge::CvImage(std_msgs::Header(), "8UC3", cv_image).toImageMsg();
    output_image->header.frame_id = "camera_depth_optical_frame";
    // output_image->header.stamp = info.header.stamp = t;
    pub.publish(output_image);
  }

private:
  ros::NodeHandle nh;
  
  ros::Publisher pub_blue_sphere;
  ros::Publisher pub_red_sphere;
  ros::Publisher pub_green_sphere;
  ros::Publisher pub_yellow_sphere;
  ros::Publisher pub_red_cylinder;
  ros::Publisher pub_green_cylinder;
  ros::Publisher pub_blue_cylinder;
  ros::Publisher pub_yellow_cylinder;

  ros::Publisher pub_blue;
  ros::Publisher pub_red;
  ros::Publisher pub_green;
  ros::Publisher pub_yellow;

  ros::Publisher image_pub_blue;
  ros::Publisher image_pub_red;
  ros::Publisher image_pub_green;
  ros::Publisher image_pub_yellow;

  ros::Publisher image_pub_blue_sphere;
  ros::Publisher image_pub_red_sphere;
  ros::Publisher image_pub_green_sphere;
  ros::Publisher image_pub_yellow_sphere;
  ros::Publisher image_pub_red_cylinder;
  ros::Publisher image_pub_green_cylinder;
  ros::Publisher image_pub_blue_cylinder;
  ros::Publisher image_pub_yellow_cylinder;

  ros::Subscriber sub_red_sphere;
  ros::Subscriber sub_green_sphere;
  ros::Subscriber sub_blue_sphere;
  ros::Subscriber sub_yellow_sphere;
  ros::Subscriber sub_red_cylinder;
  ros::Subscriber sub_green_cylinder;
  ros::Subscriber sub_blue_cylinder;
  ros::Subscriber sub_yellow_cylinder;

  ros::Subscriber sub_blue;
  ros::Subscriber sub_red;
  ros::Subscriber sub_green;
  ros::Subscriber sub_yellow;

  ros::ServiceServer service_color_filter;
  ros::ServiceServer service_shape_filter;

  ColorRange red_range;
  ColorRange green_range;
  ColorRange blue_range;
  ColorRange yellow_range;

  float red_sphere_radius;
  float red_cylinder_radius;
  float green_sphere_radius;
  float green_cylinder_radius;
  float blue_sphere_radius;
  float blue_cylinder_radius;
  float yellow_sphere_radius;
  float yellow_cylinder_radius;

  bool red_filter_success;
  bool green_filter_success;
  bool blue_filter_success;
  bool yellow_filter_success;

  bool red_sphere_success;
  bool red_cylinder_success;
  bool green_sphere_success;
  bool green_cylinder_success;
  bool blue_sphere_success;
  bool blue_cylinder_success;
  bool yellow_sphere_success;
  bool yellow_cylinder_success;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  ObjectDetection od;
  ros::spin();
}


