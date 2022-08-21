#include "best_practice_cpp_pkg/BestPractice.hpp"
#include "ros/ros.h"
namespace best_practice_cpp_pkg
{

    //! static variable initialization
    size_t BestPractice::m_number_objects{0};

    //! Constructor Initialization List
    BestPractice::BestPractice(ros::NodeHandle &t_node_handle)
        : m_is_cloud_received{false}, m_is_image_received{false},
          m_rate{100}, m_rate_waiting{1},
          m_node_handle{nullptr}, m_point_cloud{nullptr}, m_image{nullptr}
    {

        ++m_number_objects;

        m_node_handle = new ros::NodeHandle(t_node_handle);
        m_point_cloud = new sensor_msgs::PointCloud2;
        m_image = new sensor_msgs::Image;

        if (!readParameters())
        {
            ROS_ERROR("[%s] Could not read parameters.", __APP_NAME__);
            ros::requestShutdown();
        }

        ROS_INFO("[%s] Subscriber Topic Lidar: %s", __APP_NAME__, m_subscriber_topic_lidar.c_str());

        ROS_INFO("[%s] Publisher Topic Lidar: %s", __APP_NAME__, m_publisher_topic_lidar.c_str());

        ROS_INFO("[%s] Frame Id: %s", __APP_NAME__, m_frame_id.c_str());

        ROS_INFO("[%s] Subscriber Topic Image: %s", __APP_NAME__, m_subscriber_topic_image.c_str());

        ROS_INFO("[%s] Publisher Topic Image: %s", __APP_NAME__, m_publisher_topic_image.c_str());

        ROS_INFO("[%s Image Width: %d", __APP_NAME__, m_image_width);

        ROS_INFO("[%s Image Height: %d", __APP_NAME__, m_image_height);

        m_subscribers[m_subscriber_topic_lidar] =
            m_node_handle->subscribe(m_subscriber_topic_lidar, 100, &BestPractice::pointCloudCallback, this);

        m_subscribers[m_subscriber_topic_image] =
            m_node_handle->subscribe(m_subscriber_topic_image, 100, &BestPractice::imageCallback, this);

        m_publishers[m_publisher_topic_lidar] =
            m_node_handle->advertise<sensor_msgs::PointCloud2>(m_publisher_topic_lidar, 1);

        m_publishers[m_publisher_topic_image] =
            m_node_handle->advertise<sensor_msgs::Image>(m_publisher_topic_image, 1);

        ROS_INFO("[%s] Successfully launched.", __APP_NAME__);

        displayActiveObjects();

        //----------------------------------------------------------------------------------------------------------------------
        ros::Rate rate(m_rate);
        ros::Rate rate_waiting(m_rate_waiting);
        while (ros::ok())
        {
            if (areDataReceived(m_is_cloud_received, m_is_image_received))
            {
                contiuousCallback();
            }
            else
            {
                ROS_INFO("[%s] Waiting for data ...", __APP_NAME__);
                rate_waiting.sleep();
            }
            ros::spinOnce();
            rate.sleep();
        }
        //----------------------------------------------------------------------------------------------------------------------
    }

    //! Member Wise Copy Constructor Deep Coping
    BestPractice::BestPractice(const BestPractice &source)
        : BestPractice{*source.m_node_handle}
    {
        ROS_INFO("[%s] Copy Constructor is activated (DEEP COPING) .", __APP_NAME__);
    }

    //! Copy assignment operator Deep Coping
    BestPractice &BestPractice::operator=(const BestPractice &rhs)
    {
        ROS_INFO("[%s] Copy Assignment Operator is activated (DEEP COPING) .", __APP_NAME__);
        if (this == &rhs)
        {
            return *this;
            delete this->m_node_handle;
            m_node_handle = new ros::NodeHandle(*rhs.m_node_handle);
            return *this;
        }
    }

    //! Destructor Initialization .
    BestPractice::~BestPractice()
    {
        // Dealocate Memory

        delete m_point_cloud;

        delete m_node_handle;

        delete m_image;

        --m_number_objects;

        ROS_INFO("[%s]  Destructor is activated. ", __APP_NAME__);

        ros::shutdown();
    }

    inline bool BestPractice::readParameters()
    {

        if (!m_node_handle->getParam("subscriber_topic_lidar", m_subscriber_topic_lidar))
        {
            ROS_ERROR("[%s] Could not read parameter 'subscriber_topic'.", __APP_NAME__);
            return false;
        }

        if (!m_node_handle->getParam("publisher_topic_lidar", m_publisher_topic_lidar))
        {
            ROS_ERROR("[%s] Could not read parameter 'publisher_topic'.", __APP_NAME__);
            return false;
        }

        if (!m_node_handle->getParam("subscriber_topic_camera", m_subscriber_topic_image))
        {
            ROS_ERROR("[%s] Could not read parameter 'subscriber_topic'.", __APP_NAME__);
            return false;
        }

        if (!m_node_handle->getParam("publisher_topic_camera", m_publisher_topic_image))
        {
            ROS_ERROR("[%s] Could not read parameter 'publisher_topic'.", __APP_NAME__);
            return false;
        }

        if (!m_node_handle->getParam("frame_id", m_frame_id))
        {
            ROS_ERROR("[%s] Could not read parameter 'frame_id'.", __APP_NAME__);
            return false;
        }

        if (!m_node_handle->getParam("image_height", m_image_height))
        {
            ROS_ERROR("[%s] Could not read parameter 'image_height'.", __APP_NAME__);
            return false;
        }

        if (!m_node_handle->getParam("image_width", m_image_width))
        {
            ROS_ERROR("[%s] Could not read parameter 'image_width'.", __APP_NAME__);
            return false;
        }

        return true;
    }

    inline bool BestPractice::areDataReceived(const bool &is_cloud_received, const bool &is_image_received)
    {
        if (is_cloud_received && is_image_received)
        {
            ROS_INFO("[%s] Received data from all subscribers.", __APP_NAME__);
            return true;
        }
        return false;
    }

    inline size_t BestPractice::objectCounter()
    {
        return m_number_objects;
    }

    inline void BestPractice::displayActiveObjects() const
    {
        ROS_INFO("[%s] Number of active objects: %d", __APP_NAME__, m_number_objects);
    }

    inline void BestPractice::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &t_point_cloud)
    {
        m_is_cloud_received = true;
        *m_point_cloud = *t_point_cloud;
    }
    inline void BestPractice::imageCallback(const sensor_msgs::ImageConstPtr &t_image)
    {
        m_is_image_received = true;
        *m_image = *t_image;
    }

    inline void BestPractice::contiuousCallback()
    {

        //! Build Your Algorithm Here

        m_publishers[m_publisher_topic_lidar].publish(*m_point_cloud);

        m_publishers[m_publisher_topic_image].publish(*m_image);
    }

}