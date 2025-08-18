#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/conditional_removal.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>

#include "std_srvs/srv/set_bool.hpp"

#include <string>
#include <memory>

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

class ObjectDetection : public rclcpp::Node
{
public:
  ObjectDetection() : Node("object_detection")
  {
    // Publishers for point clouds
    pub_green_sphere = this->create_publisher<sensor_msgs::msg::PointCloud2>("/green_sphere", 1);
    pub_red_sphere = this->create_publisher<sensor_msgs::msg::PointCloud2>("/red_sphere", 1);
    pub_blue_sphere = this->create_publisher<sensor_msgs::msg::PointCloud2>("/blue_sphere", 1);
    pub_yellow_sphere = this->create_publisher<sensor_msgs::msg::PointCloud2>("/yellow_sphere", 1);
    pub_green_cylinder = this->create_publisher<sensor_msgs::msg::PointCloud2>("/green_cylinder", 1);
    pub_red_cylinder = this->create_publisher<sensor_msgs::msg::PointCloud2>("/red_cylinder", 1);
    pub_blue_cylinder = this->create_publisher<sensor_msgs::msg::PointCloud2>("/blue_cylinder", 1);
    pub_yellow_cylinder = this->create_publisher<sensor_msgs::msg::PointCloud2>("/yellow_cylinder", 1);

    pub_blue = this->create_publisher<sensor_msgs::msg::PointCloud2>("/blue_filter", 1);
    pub_red = this->create_publisher<sensor_msgs::msg::PointCloud2>("/red_filter", 1);
    pub_green = this->create_publisher<sensor_msgs::msg::PointCloud2>("/green_filter", 1);
    pub_yellow = this->create_publisher<sensor_msgs::msg::PointCloud2>("/yellow_filter", 1);

    // Publishers for images
    image_pub_blue = this->create_publisher<sensor_msgs::msg::Image>("blue_filtered_image", 1);
    image_pub_green = this->create_publisher<sensor_msgs::msg::Image>("green_filtered_image", 1);
    image_pub_red = this->create_publisher<sensor_msgs::msg::Image>("red_filtered_image", 1);
    image_pub_yellow = this->create_publisher<sensor_msgs::msg::Image>("yellow_filtered_image", 1);

    image_pub_green_sphere = this->create_publisher<sensor_msgs::msg::Image>("/green_sphere_image", 1);
    image_pub_red_sphere = this->create_publisher<sensor_msgs::msg::Image>("/red_sphere_image", 1);
    image_pub_blue_sphere = this->create_publisher<sensor_msgs::msg::Image>("/blue_sphere_image", 1);
    image_pub_yellow_sphere = this->create_publisher<sensor_msgs::msg::Image>("/yellow_sphere_image", 1);
    image_pub_green_cylinder = this->create_publisher<sensor_msgs::msg::Image>("/green_cylinder_image", 1);
    image_pub_red_cylinder = this->create_publisher<sensor_msgs::msg::Image>("/red_cylinder_image", 1);
    image_pub_blue_cylinder = this->create_publisher<sensor_msgs::msg::Image>("/blue_cylinder_image", 1);
    image_pub_yellow_cylinder = this->create_publisher<sensor_msgs::msg::Image>("/yellow_cylinder_image", 1);

    // Services (using standard services for now - you can create custom ones later)
    service_color_filter = this->create_service<std_srvs::srv::SetBool>(
        "color_filter_server", 
        std::bind(&ObjectDetection::call_color_filter_simple, this, std::placeholders::_1, std::placeholders::_2));
    
    service_shape_filter = this->create_service<std_srvs::srv::SetBool>(
        "shape_filter_server", 
        std::bind(&ObjectDetection::call_shape_filter_simple, this, std::placeholders::_1, std::placeholders::_2));

    // Transform broadcaster
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize default parameters
    red_sphere_radius = 0.1;
    green_sphere_radius = 0.1;
    blue_sphere_radius = 0.1;
    yellow_sphere_radius = 0.1;
    red_cylinder_radius = 0.1;
    green_cylinder_radius = 0.1;
    blue_cylinder_radius = 0.1;
    yellow_cylinder_radius = 0.1;

    // Initialize color ranges with default values
    assign_color_range(red_range, 255, 200, 100, 0, 100, 0);
    assign_color_range(green_range, 100, 0, 255, 200, 100, 0);
    assign_color_range(blue_range, 255, 0, 255, 0, 255, 30); 
    assign_color_range(yellow_range, 255, 200, 255, 200, 100, 0);

    RCLCPP_INFO(this->get_logger(), "Object Detection Node Started");
  }

  // Simplified service callbacks using standard SetBool service
  void call_color_filter_simple(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if(req->data == true){
      // Start all color filters with default parameters
      sub_red = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/base_camera/points", 1, 
          std::bind(&ObjectDetection::redfilter_callback, this, std::placeholders::_1));
      sub_green = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/base_camera/points", 1, 
          std::bind(&ObjectDetection::greenfilter_callback, this, std::placeholders::_1));
      sub_blue = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/base_camera/points", 1, 
          std::bind(&ObjectDetection::bluefilter_callback, this, std::placeholders::_1));
      sub_yellow = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/base_camera/points", 1, 
          std::bind(&ObjectDetection::yellowfilter_callback, this, std::placeholders::_1));
      res->success = true;
      res->message = "Color filters started";
    }
    else{
      sub_red.reset();
      sub_green.reset();
      sub_blue.reset();
      sub_yellow.reset();
      res->success = true;
      res->message = "Color filters stopped";
    }
  }

  void call_shape_filter_simple(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                               std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if(req->data == true){
      // Start all shape detection with default radius
      sub_red_sphere = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/red_filter", 1, 
          std::bind(&ObjectDetection::red_sphere_callback, this, std::placeholders::_1));
      sub_green_sphere = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/green_filter", 1, 
          std::bind(&ObjectDetection::green_sphere_callback, this, std::placeholders::_1));
      sub_blue_sphere = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/blue_filter", 1, 
          std::bind(&ObjectDetection::blue_sphere_callback, this, std::placeholders::_1));
      sub_yellow_sphere = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/yellow_filter", 1, 
          std::bind(&ObjectDetection::yellow_sphere_callback, this, std::placeholders::_1));
      
      sub_red_cylinder = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/red_filter", 1, 
          std::bind(&ObjectDetection::red_cylinder_callback, this, std::placeholders::_1));
      sub_green_cylinder = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/green_filter", 1, 
          std::bind(&ObjectDetection::green_cylinder_callback, this, std::placeholders::_1));
      sub_blue_cylinder = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/blue_filter", 1, 
          std::bind(&ObjectDetection::blue_cylinder_callback, this, std::placeholders::_1));
      sub_yellow_cylinder = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/yellow_filter", 1, 
          std::bind(&ObjectDetection::yellow_cylinder_callback, this, std::placeholders::_1));
      
      res->success = true;
      res->message = "Shape detection started";
    }
    else{
      sub_red_sphere.reset();
      sub_green_sphere.reset();
      sub_blue_sphere.reset();
      sub_yellow_sphere.reset();
      sub_red_cylinder.reset();
      sub_green_cylinder.reset();
      sub_blue_cylinder.reset();
      sub_yellow_cylinder.reset();
      res->success = true;
      res->message = "Shape detection stopped";
    }
  }

  void assign_color_range(ColorRange &color_range, int rMax, int rMin, int gMax, int gMin, int bMax, int bMin){
    color_range.rMax = rMax;
    color_range.rMin = rMin;
    color_range.gMax = gMax;
    color_range.gMin = gMin;
    color_range.bMax = bMax;
    color_range.bMin = bMin;
  }

  void yellowfilter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloudRGB::Ptr cloud_input(new PointCloudRGB);
    pcl::fromROSMsg(*msg, *cloud_input);
    
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, yellow_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, yellow_range.bMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, yellow_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, yellow_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, yellow_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, yellow_range.gMin)));

    // Build the filter
    color_filter.setInputCloud(cloud_input);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_color_filtered, output_msg);
    output_msg.header = msg->header;
    pub_yellow->publish(output_msg);
    pointcloud_to_rgb_image(cloud_color_filtered, image_pub_yellow, msg->header);
  }

  void bluefilter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "=== BLUE FILTER CALLBACK CALLED ===");
    
    PointCloudRGB::Ptr cloud_input(new PointCloudRGB);
    pcl::fromROSMsg(*msg, *cloud_input);
    
    RCLCPP_INFO(this->get_logger(), "Input cloud size: %zu", cloud_input->points.size());
    
    // Sample first few points to see actual RGB values
    for(size_t i = 0; i < std::min(cloud_input->points.size(), size_t(10)); i++) {
        auto& pt = cloud_input->points[i];
        if(!std::isnan(pt.x)) {
            RCLCPP_INFO(this->get_logger(), "Point %zu: r=%d, g=%d, b=%d", i, pt.r, pt.g, pt.b);
        }
    }
    
    RCLCPP_INFO(this->get_logger(), "Blue range: r(%d-%d), g(%d-%d), b(%d-%d)", 
                blue_range.rMin, blue_range.rMax, 
                blue_range.gMin, blue_range.gMax, 
                blue_range.bMin, blue_range.bMax);
    
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, blue_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, blue_range.bMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, blue_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, blue_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, blue_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, blue_range.gMin)));

    // Build the filter
    color_filter.setInputCloud(cloud_input);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    RCLCPP_INFO(this->get_logger(), "Filtered cloud size: %zu", cloud_color_filtered->points.size());

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_color_filtered, output_msg);
    output_msg.header = msg->header;
    pub_blue->publish(output_msg);
    pointcloud_to_rgb_image(cloud_color_filtered, image_pub_blue, msg->header);
    
    RCLCPP_INFO(this->get_logger(), "=== END BLUE FILTER DEBUG ===");
}

  void redfilter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloudRGB::Ptr cloud_input(new PointCloudRGB);
    pcl::fromROSMsg(*msg, *cloud_input);
    
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, red_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, red_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, red_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, red_range.gMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, red_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, red_range.bMin)));

    // Build the filter
    color_filter.setInputCloud(cloud_input);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_color_filtered, output_msg);
    output_msg.header = msg->header;
    pub_red->publish(output_msg);
    pointcloud_to_rgb_image(cloud_color_filtered, image_pub_red, msg->header);
  }

  void greenfilter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloudRGB::Ptr cloud_input(new PointCloudRGB);
    pcl::fromROSMsg(*msg, *cloud_input);
    
    PointCloudRGB::Ptr cloud_color_filtered(new PointCloudRGB);

    pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;

    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::LT, green_range.gMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("g", pcl::ComparisonOps::GT, green_range.gMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::LT, green_range.rMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("r", pcl::ComparisonOps::GT, green_range.rMin)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::LT, green_range.bMax)));
    color_cond->addComparison (pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr (new pcl::PackedRGBComparison<pcl::PointXYZRGB> ("b", pcl::ComparisonOps::GT, green_range.bMin)));

    // Build the filter
    color_filter.setInputCloud(cloud_input);
    color_filter.setCondition (color_cond);
    color_filter.filter(*cloud_color_filtered);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_color_filtered, output_msg);
    output_msg.header = msg->header;
    pub_green->publish(output_msg);
    pointcloud_to_rgb_image(cloud_color_filtered, image_pub_green, msg->header);
  }

  void green_sphere_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_sphere(msg, pub_green_sphere, image_pub_green_sphere, "green_sphere", green_sphere_radius);
  }

  void red_sphere_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_sphere(msg, pub_red_sphere, image_pub_red_sphere, "red_sphere", red_sphere_radius);
  }

  void blue_sphere_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_sphere(msg, pub_blue_sphere, image_pub_blue_sphere, "blue_sphere", blue_sphere_radius);
  }

  void yellow_sphere_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_sphere(msg, pub_yellow_sphere, image_pub_yellow_sphere, "yellow_sphere", yellow_sphere_radius);
  }

  void green_cylinder_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_cylinder(msg, pub_green_cylinder, image_pub_green_cylinder, "green_cylinder", green_cylinder_radius);
  }

  void red_cylinder_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_cylinder(msg, pub_red_cylinder, image_pub_red_cylinder, "red_cylinder", red_cylinder_radius);
  }

  void blue_cylinder_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_cylinder(msg, pub_blue_cylinder, image_pub_blue_cylinder, "blue_cylinder", blue_cylinder_radius);
  }

  void yellow_cylinder_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    detect_cylinder(msg, pub_yellow_cylinder, image_pub_yellow_cylinder, "yellow_cylinder", yellow_cylinder_radius);
  }

  void detect_cylinder(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, 
                      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, 
                      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub, 
                      std::string frame_id, float radius)
  {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

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
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1);
    pass.filter (*cloud_filtered);

    if(cloud_filtered->points.size() < 10){
      RCLCPP_INFO(this->get_logger(), "Cannot filter out desired shape");
      return;
    }

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

    if(cloud_filtered2->points.size() < 10){
      RCLCPP_INFO(this->get_logger(), "Cannot filter out desired shape");
      return;
    }

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

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // Publish transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = cloud_msg->header.frame_id;
    t.child_frame_id = frame_id;
    t.transform.translation.x = coefficients_cylinder->values[0];
    t.transform.translation.y = coefficients_cylinder->values[1];
    t.transform.translation.z = coefficients_cylinder->values[2];
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(t);

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_cylinder, output_msg);
    output_msg.header = cloud_msg->header;
    pub->publish(output_msg);
    
    pointcloud_to_depth_image(cloud_cylinder, image_pub, cloud_msg->header);
  }

  void detect_sphere(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, 
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, 
                    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub, 
                    std::string frame_id, float radius)
  {
    PointCloud::Ptr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

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
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1);
    pass.filter (*cloud_filtered);

    if(cloud_filtered->points.size() < 1000){
      RCLCPP_INFO(this->get_logger(), "Cannot filter out desired shape");
      return;
    }

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

    if(cloud_filtered2->points.size() < 1000){
      RCLCPP_INFO(this->get_logger(), "Cannot filter out desired shape");
      return;
    }

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

    // Publish transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = cloud_msg->header.frame_id;
    t.child_frame_id = frame_id;
    t.transform.translation.x = coefficients_sphere->values[0];
    t.transform.translation.y = coefficients_sphere->values[1];
    t.transform.translation.z = coefficients_sphere->values[2];
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster->sendTransform(t);

    // Write the sphere inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_sphere);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_sphere);

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_sphere, output_msg);
    output_msg.header = cloud_msg->header;
    pub->publish(output_msg);
    
    pointcloud_to_depth_image(cloud_sphere, image_pub, cloud_msg->header);
  }

  void pointcloud_to_depth_image(const PointCloud::ConstPtr& msg, 
                                rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
                                const std_msgs::msg::Header& header)
  {
    float centre_x = 320.5;
    float centre_y = 240.5;
    float focal_x = 554.254691191187;
    float focal_y = 554.254691191187;
    int height = 480;
    int width = 640;

    cv::Mat cv_image = cv::Mat(height, width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

    for (size_t i = 0; i < msg->points.size(); i++){
      if (!std::isnan(msg->points[i].z)){
        float z = msg->points[i].z * 1000.0;
        float u = (msg->points[i].x * 1000.0 * focal_x) / z;
        float v = (msg->points[i].y * 1000.0 * focal_y) / z;
        int pixel_pos_x = static_cast<int>(u + centre_x);
        int pixel_pos_y = static_cast<int>(v + centre_y);

        if (pixel_pos_x >= width){
          pixel_pos_x = width - 1;
        }
        if (pixel_pos_x < 0){
          pixel_pos_x = 0;
        }
        if (pixel_pos_y >= height){
          pixel_pos_y = height - 1;
        }
        if (pixel_pos_y < 0){
          pixel_pos_y = 0;
        }
        cv_image.at<float>(pixel_pos_y, pixel_pos_x) = z;
      }       
    }

    cv_image.convertTo(cv_image, CV_8UC1);

    auto output_image = cv_bridge::CvImage(header, "mono8", cv_image).toImageMsg();
    output_image->header.frame_id = "camera_depth_optical_frame";
    pub->publish(*output_image);
  }

  void pointcloud_to_rgb_image(const PointCloudRGB::ConstPtr& msg, 
                              rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub,
                              const std_msgs::msg::Header& header)
  {
    float centre_x = 320.5;
    float centre_y = 240.5;
    float focal_x = 554.254691191187;
    float focal_y = 554.254691191187;
    int height = 480;
    int width = 640;

    cv::Mat cv_image = cv::Mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    for (size_t i = 0; i < msg->points.size(); i++){
      if (!std::isnan(msg->points[i].z)){
        float z = msg->points[i].z * 1000.0;
        float u = (msg->points[i].x * 1000.0 * focal_x) / z;
        float v = (msg->points[i].y * 1000.0 * focal_y) / z;
        int pixel_pos_x = static_cast<int>(u + centre_x);
        int pixel_pos_y = static_cast<int>(v + centre_y);

        int r = msg->points[i].r;
        int g = msg->points[i].g;
        int b = msg->points[i].b;

        if (pixel_pos_x >= width){
          pixel_pos_x = width - 1;
        }
        if (pixel_pos_x < 0){
          pixel_pos_x = 0;
        }
        if (pixel_pos_y >= height){
          pixel_pos_y = height - 1;
        }
        if (pixel_pos_y < 0){
          pixel_pos_y = 0;
        }

        cv_image.at<cv::Vec3b>(pixel_pos_y, pixel_pos_x) = cv::Vec3b(b, g, r);
      }       
    }

    auto output_image = cv_bridge::CvImage(header, "bgr8", cv_image).toImageMsg();
    output_image->header.frame_id = "camera_depth_optical_frame";
    pub->publish(*output_image);
  }

private:
  // Publishers for point clouds
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_blue_sphere;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_red_sphere;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_green_sphere;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_yellow_sphere;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_red_cylinder;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_green_cylinder;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_blue_cylinder;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_yellow_cylinder;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_blue;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_red;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_green;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_yellow;

  // Publishers for images
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_blue;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_red;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_green;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_yellow;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_blue_sphere;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_red_sphere;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_green_sphere;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_yellow_sphere;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_red_cylinder;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_green_cylinder;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_blue_cylinder;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_yellow_cylinder;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_red_sphere;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_green_sphere;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_blue_sphere;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_yellow_sphere;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_red_cylinder;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_green_cylinder;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_blue_cylinder;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_yellow_cylinder;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_blue;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_red;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_green;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_yellow;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_color_filter;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_shape_filter;

  // Transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  // Color ranges
  ColorRange red_range;
  ColorRange green_range;
  ColorRange blue_range;
  ColorRange yellow_range;

  // Radius parameters
  float red_sphere_radius;
  float red_cylinder_radius;
  float green_sphere_radius;
  float green_cylinder_radius;
  float blue_sphere_radius;
  float blue_cylinder_radius;
  float yellow_sphere_radius;
  float yellow_cylinder_radius;

  // Status flags
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}