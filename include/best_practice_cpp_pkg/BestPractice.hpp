#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/Image.h"

#define __APP_NAME__ "best_practice"

namespace best_practice_cpp_pkg
{

    /*!
     * Main class for the node to handle the ROS interfacing.
     */

    class BestPractice
    {
    public:
        /*!
         * Constructor.
         * @param t_node_handle the ROS node handle.
         */

        explicit BestPractice(ros::NodeHandle &t_node_handle);

        /*!
         *  Copy Constructor.
         */
        BestPractice(const BestPractice &source);

        /*!
         *  Overload Copy Constructor and it is assigned to the current object.
         */
        BestPractice &operator=(const BestPractice &rhs);

        /*!
         * Destructor.
         */
        virtual ~BestPractice();

    private:
        /*!
         * Reads and verifies the ROS parameters.
         * @return true if successful.
         */
        inline bool readParameters();

        /*!
         * Reads and verifies the ROS subscribed to data.
         * @return true if successful.
         */
        inline bool areDataReceived(const bool &is_cloud_received, const bool &is_image_received);

        /*!
         * CounterObjects.
         * @return size of how many object is created  if successful.
         */
        static inline size_t objectCounter();

        /*!
         * DisplayActiveObjects.
         * @return  if successful.
         */
        inline void displayActiveObjects() const;

        /*!
         * ROS topic callback method.
         * @param t_point_cloud the received message.
         */
        inline void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &t_point_cloud);

        /*!
         * ROS topic callback method.
         * @param t_image the received message.
         */
        inline void imageCallback(const sensor_msgs::ImageConstPtr &t_image);

        /*!
         * ROS topic Constructor callback method.
         * @param m_point_cloud @param m_image  the received message.
         */
        inline void contiuousCallback();

        //! ROS node handle.
        ros::NodeHandle *m_node_handle;

        //! ROS pointer sensor_msgs::pointcloud2 .
        sensor_msgs::PointCloud2 *m_point_cloud;

        //! ROS pointer sensor_msgs::image.
        sensor_msgs::Image *m_image;

        // Publishers mao varriable.
        std::unordered_map<std::string, ros::Publisher> m_publishers;

        // Subscribers mao varriable.
        std::unordered_map<std::string, ros::Subscriber> m_subscribers;

        //! ROS subscriber topic name .
        std::string m_subscriber_topic_lidar;

        //! ROS publisher topic name.
        std::string m_publisher_topic_lidar;

        //! ROS subscriber topic name .
        std::string m_subscriber_topic_image;

        //! ROS publisher topic name.
        std::string m_publisher_topic_image;

        //! ROS frame id.
        std::string m_frame_id;

        //! Get the number of objects.
        static size_t m_number_objects;

        //! image width.
        int m_image_width;

        //! m_image height.
        int m_image_height;

        //! Flag for receiving point cloud.
        bool m_is_cloud_received{};

        //! Flag for receiving image.
        bool m_is_image_received{};

        //! value of rate for contiuousCallback.
        const double m_rate{};

        //! value of rate waiting for contiuousCallback.
        const double m_rate_waiting{};
    };
} // namespace best_practice_cpp_pkg
