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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointXYZ PointT;

class ObjectDetection
{
public:
  ObjectDetection()
  {
    pub_green_sphere = nh.advertise<PointCloud> ("/filter1", 1);
    pub_red_sphere = nh.advertise<PointCloud> ("/filter2", 1);
    pub_blue_sphere = nh.advertise<PointCloud> ("/filter3", 1);
    pub_green_cylinder = nh.advertise<PointCloud> ("/filter4", 1);
    pub_blue = nh.advertise<PointCloudRGB> ("/blue_filter", 1);
    pub_red = nh.advertise<PointCloudRGB> ("/red_filter", 1);
    pub_green = nh.advertise<PointCloudRGB> ("/green_filter", 1);

    sub_green_sphere = nh.subscribe<PointCloud>("/green_filter", 1, &ObjectDetection::green_sphere_callback, this);
    sub_red_sphere = nh.subscribe<PointCloud>("/red_filter", 1, &ObjectDetection::red_sphere_callback, this);
    sub_blue_sphere = nh.subscribe<PointCloud>("/blue_filter", 1, &ObjectDetection::blue_sphere_callback, this);
    sub_green_cylinder = nh.subscribe<PointCloud>("/green_filter", 1, &ObjectDetection::green_cylinder_callback, this);
    sub_blue = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::bluefilter_callback, this);
    sub_red = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::redfilter_callback, this);
    sub_green = nh.subscribe<PointCloudRGB>("/kinect_camera_fixed/depth/points", 1, &ObjectDetection::greenfilter_callback, this);
  }

  void bluefilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    int bMax = 255;
    int bMin = 90;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, bMin)));


    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_blue.publish(cloud_color_filtered);
  }

  void redfilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    // build the condition
    int rMax = 255;
    int rMin = 90;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, rMin)));

    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_red.publish(cloud_color_filtered);
  }

  void greenfilter_callback(const PointCloudRGB::ConstPtr& msg)
  {
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);
    pcl::PCLPointCloud2 cloud_filtered_ros;

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    // build the condition
    int gMax = 255;
    int gMin = 90;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, gMin)));

    // Build the filter
    color_filter.setInputCloud(msg);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    pub_green.publish(cloud_color_filtered);
  }

  void green_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    //   printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    detect_sphere(msg, pub_green_sphere, "green_sphere");
  }

  void red_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    detect_sphere(msg, pub_red_sphere, "red_sphere");
  }

  void blue_sphere_callback(const PointCloud::ConstPtr& msg)
  {
    detect_sphere(msg, pub_blue_sphere, "blue_sphere");
  }

  void green_cylinder_callback(const PointCloud::ConstPtr& msg)
  {
    detect_cylinder(msg, pub_green_cylinder, "green_cylinder");
  }

  void detect_cylinder(const PointCloud::ConstPtr& cloud, ros::Publisher pub, std::string frame_id)
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
    seg.setRadiusLimits (0, 0.05);
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
  }

  void detect_sphere(const PointCloud::ConstPtr& cloud, ros::Publisher pub, std::string frame_id)
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
    seg.setRadiusLimits (0, 0.05);
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
  }

private:
  ros::NodeHandle nh;
  
  ros::Publisher pub_blue_sphere;
  ros::Publisher pub_red_sphere;
  ros::Publisher pub_green_sphere;
  ros::Publisher pub_green_cylinder;
  ros::Publisher pub_blue;
  ros::Publisher pub_red;
  ros::Publisher pub_green;

  ros::Subscriber sub_red_sphere;
  ros::Subscriber sub_green_sphere;
  ros::Subscriber sub_blue_sphere;
  ros::Subscriber sub_green_cylinder;
  ros::Subscriber sub_blue;
  ros::Subscriber sub_red;
  ros::Subscriber sub_green;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection");
  ObjectDetection od;
  ros::spin();
}


