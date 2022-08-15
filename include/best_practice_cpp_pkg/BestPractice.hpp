#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

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

        BestPractice(ros::NodeHandle &t_node_handle);

        /*!
         *  Copy Constructor.
         */
        BestPractice(const BestPractice &source);

        /*!
         * Destructor.
         */
        ~BestPractice();


        /*!
         * CounterObjects.
         * @return size of how many object is created  if successful.
         */
        static inline size_t CounterObject();


        /*!
         * DisplayActiveObjects.
         * @return  if successful.
         */
        inline void DisplayActiveObjects() const;

        static size_t m_number_objects;

    private:
        /*!
         * Reads and verifies the ROS parameters.
         * @return true if successful.
         */
        inline bool readParameters();

        /*!
         * ROS topic callback method.
         * @param t_point_cloud the received message.
         */
        inline void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &t_point_cloud);

        /*!
         * ROS topic Constructor callback method.
         * @param c_point_cloud the received message.
         */
        inline void contiuousPointCloudCallback();

        //! ROS node handle.
        ros::NodeHandle *m_node_handle;

        //! ROS pointer sensor_msgs::pointcloud2 .
        sensor_msgs::PointCloud2 *m_point_cloud;

        //! ROS publisher for the output point cloud.
        ros::Publisher m_publisher;

        //! ROS subscriber for the input point cloud.
        ros::Subscriber m_subscriber;

        //! ROS subscriber topic name .
        std::string m_subscriber_topic;

        //! ROS publisher topic name.
        std::string m_publisher_topic;

        //! ROS frame id.
        std::string m_frame_id;

        //! Flag for receiving point cloud.
        bool m_is_cloud_received{};

        //! value of rate for contiuousPointCloudCallback.
        const double m_rate{};

        //! value of rate for contiuousPointCloudCallback.
        const double m_rate_waiting{};
    };
} // namespace best_practice_cpp_pkg
