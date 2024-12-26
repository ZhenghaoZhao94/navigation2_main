#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

class MyLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    MyLifecycleNode() : rclcpp_lifecycle::LifecycleNode("my_lifecycle_node") {}
    // 配置状态回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Configuring node...");
        publisher_ = std::make_shared<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>>(
            this->get_node_base_interface(),
            this->get_node_topics_interface(),
            this->get_node_services_interface(),
            "my_topic",
            rclcpp::QoS(10));
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    // 激活状态回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Activating node...");
        publisher_->on_activate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    // 停用状态回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Deactivating node...");
        publisher_->on_deactivate();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    // 清理状态回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Cleaning up node...");
        publisher_.reset();
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
    // 关闭状态回调函数
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&)
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down node...");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
};
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyLifecycleNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}




 /*
     需求：红绿灯生命周期
     流程：
         1.导包；
         2.初始化ROS2客户端；
         3.自定义节点类；
         4.调用spain函数，并传入节点对象；
         5.资源释放。
 */
 #include <rclcpp/rclcpp.hpp>
 #include <cv_bridge/cv_bridge.h>
 #include <image_transport/image_transport.hpp>
 #include <sensor_msgs/msg/image.hpp>
 #include <std_msgs/msg/bool.hpp>
 #include <opencv2/opencv.hpp>
 #include <rclcpp_lifecycle/lifecycle_node.hpp>
 #include <rclcpp_lifecycle/lifecycle_publisher.hpp>
 ​
 using std::placeholders::_1;
 using namespace cv;
 // 3.定义节点类；
 class traffic_light_status : public rclcpp_lifecycle::LifecycleNode
 {
 public:
     traffic_light_status() : rclcpp_lifecycle::LifecycleNode("traffic_light_status")
     {
         // 动态调参
         this->declare_parameter<int>("red_threshold_", 120);
         this->declare_parameter<int>("roi_x", 220);
         this->declare_parameter<int>("roi_y", 150);
         this->declare_parameter<int>("roi_width", 420);
         this->declare_parameter<int>("roi_hight", 190);
         this->declare_parameter<int>("size_x", 20);
         this->declare_parameter<int>("size_y", 20);
     }
     // 当节点配置时调用回调函数
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
     {
         // rclcpp_lifecycle::LifecycleNode到rclcpp::Node的转换，使用shared_from_this()提供的智能指针直接转换，因为LifecycleNode是从Node继承来的
         // 先转换为rclcpp::Node*，然后创建一个新的shared_ptr。
         // 先获得裸指针，再用裸指针创建对应的std::shared_ptr<rclcpp::Node>。
         // 使用dynamic_cast进行安全转换，确保转换过程中的类型安全。如果dynamic_cast失败（即如果this不是一个rclcpp::Node），它将返回nullptr。
         auto raw_node_ptr = dynamic_cast<rclcpp::Node *>(this->get_node_base_interface().get());
         if (!raw_node_ptr)
         {
             RCLCPP_ERROR(this->get_logger(), "rclcpp::Node*转换失败！！");
             return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
         }
         auto node_shared_ptr = std::shared_ptr<rclcpp::Node>(raw_node_ptr);
         auto it = std::make_shared<image_transport::ImageTransport>(node_shared_ptr);
         image_sub_ = it->subscribe("camera2/image_raw", 1, std::bind(&traffic_light_status::imageCallback, this, _1));
         traffic_light_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/traffic_light_status", 10);
 
         red_threshold_ = this->get_parameter("red_threshold_").as_int();
         roi_x = this->get_parameter("roi_x").as_int();
         roi_y = this->get_parameter("roi_y").as_int();
         roi_width = this->get_parameter("roi_width").as_int();
         roi_hight = this->get_parameter("roi_hight").as_int();
         size_x = this->get_parameter("size_x").as_int();
         size_y = this->get_parameter("size_y").as_int();
 
         RCLCPP_INFO(this->get_logger(), "TrafficLightStatus node configured");
         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }
     // 当节点激活时调用的回调函数
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
     {
         traffic_light_publisher_->on_activate();
         RCLCPP_INFO(this->get_logger(), "TrafficLightStatus node activated");
         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }
     // 当节点停用时调用回调函数
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
     {
         traffic_light_publisher_->on_deactivate();
         RCLCPP_INFO(this->get_logger(), "TrafficLightStatus node deactivated");
         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }
     // 当节点清理时调用回调函数
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
     {
         image_sub_.shutdown();
         traffic_light_publisher_.reset();
         RCLCPP_INFO(this->get_logger(), "TrafficLightStatus node cleaned up");
         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }
     // 当节点关闭时调用的回调函数
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
     {
         RCLCPP_INFO(this->get_logger(), "TrafficLightStatus node shutting down");
         return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
     }
 
 private:
     image_transport::Subscriber image_sub_;
     rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr traffic_light_publisher_;
 
     // 动态调参
     int red_threshold_;
     int roi_x;
     int roi_y;
     int roi_width;
     int roi_hight;
     int size_x;
     int size_y;
 
     void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
     {
         cv_bridge::CvImagePtr cv_ptr;
         try
         {
             cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
             Mat frame = cv_ptr->image;
 
             // 检测红灯发布状态
             std_msgs::msg::Bool light_status_msg;
             light_status_msg.data = detectRedLight(frame);
             traffic_light_publisher_->publish(light_status_msg);
             waitKey(1);
         }
         catch (const cv_bridge::Exception &e)
         {
             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
         }
     }
     bool detectRedLight(const Mat &img)
     {
         Mat copy_img = img.clone(); // 复制一张原图
         // 裁剪图像
         cv::Rect roiRect(roi_x, roi_y, roi_width, roi_hight);
         // 对裁剪后的图像进行处理
         cv::Mat frame_roi = copy_img(roiRect);
         // imshow("红绿灯",frame_roi);
         Mat bgr_channels[3];                                                             // 分离图像通道
         split(frame_roi, bgr_channels);                                                  // 分离BGR通道
         Mat red_minus_green;                                                             // 红色通道减去绿色通道的结果
         subtract(bgr_channels[2], bgr_channels[1], red_minus_green);                     // 计算红色减去绿色的结果
         threshold(red_minus_green, red_minus_green, red_threshold_, 255, THRESH_BINARY); // 应用阈值
         Mat element = getStructuringElement(MORPH_RECT, Size(size_x, size_y));           // 结构化元素
         dilate(red_minus_green, red_minus_green, element);                               // 膨胀操作
         // imshow("红绿灯二值图",red_minus_green);
         std::vector<std::vector<Point>> contours;                                // 存储轮廓
         findContours(red_minus_green, contours, RETR_TREE, CHAIN_APPROX_SIMPLE); // 寻找轮廓
                                                                                  // 计算最大红色区域的大小
         double maxArea = 0;
         for (const auto &contour : contours)
         {
             double area = contourArea(contour); // 计算每个轮廓的面积
             std::cout << "area" << area << std::endl;
             if (area > maxArea)
             {
                 maxArea = area; // 更新最大面积
             }
         }
         const double AREA_THRESHOLD = 1200; // 设定面积阈值
         if (maxArea > AREA_THRESHOLD)       // 如果最大面积超过阈值
         {
             Mat red_detection_result = Mat::zeros(img.size(), CV_8UC3);             // 创建结果图像
             drawContours(red_detection_result, contours, -1, Scalar(0, 0, 255), 3); // 绘制轮廓
             imshow("红灯检测", red_detection_result);                               // 显示检测结果
             waitKey(1);                                                             // 等待1毫秒
             return true;                                                            // 返回真，表示检测到红色信号
         }
         return false;
     }
 };
 
 int main(int argc, char *argv[])
 {
     // 2.初始化 ROS2 客户端；
     rclcpp::init(argc, argv);
     // 4.调用spin函数，并传入节点对象指针。
     auto node = std::make_shared<traffic_light_status>();
     rclcpp::spin(node->get_node_base_interface());
     // 5.释放资源；
     rclcpp::shutdown();
     return 0;
 }