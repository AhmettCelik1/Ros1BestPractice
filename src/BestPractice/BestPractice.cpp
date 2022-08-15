#include "best_practice_cpp_pkg/BestPractice.hpp"
#include "ros/ros.h"
namespace best_practice_cpp_pkg
{

    //! static variable initialization
    size_t BestPractice::m_number_objects{0};

    //! Constructor Initialization List

    BestPractice::BestPractice(ros::NodeHandle &t_node_handle)
        : m_node_handle{nullptr}, m_is_cloud_received{false},
          m_rate{20}, m_rate_waiting{1}, m_point_cloud{nullptr}
    {

        ++m_number_objects;

        m_node_handle = new ros::NodeHandle(t_node_handle);
        m_point_cloud = new sensor_msgs::PointCloud2;

        if (!readParameters())
        {
            ROS_ERROR("[%s] Could not read parameters.", __APP_NAME__);
            ros::requestShutdown();
        }

        ROS_INFO("[%s] Subscriber Topic: %s", __APP_NAME__, m_subscriber_topic.c_str());
        ROS_INFO("[%s] Publisher Topic: %s", __APP_NAME__, m_publisher_topic.c_str());
        ROS_INFO("[%s] Frame Id: %s", __APP_NAME__, m_frame_id.c_str());

        m_subscriber = m_node_handle->subscribe(m_subscriber_topic, 1, &BestPractice::pointCloudCallback, this);
        m_publisher = m_node_handle->advertise<sensor_msgs::PointCloud2>(m_publisher_topic, 1);

        ROS_INFO("[%s] Successfully launched.", __APP_NAME__);

        DisplayActiveObjects();

        //----------------------------------------------------------------------------------------------------------------------
        // ros::Rate rate(m_rate);
        // ros::Rate rate_waiting(m_rate_waiting);
        // while (ros::ok())
        // {
        //     if (m_is_cloud_received)
        //     {
        //         contiuousPointCloudCallback();
        //     }
        //     else
        //     {
        //         ROS_INFO("[%s] Waiting for cloud...", __APP_NAME__);
        //         rate_waiting.sleep();
        //     }
        //     ros::spinOnce();
        //     rate.sleep();
        // }
        //----------------------------------------------------------------------------------------------------------------------
    }

    //! Member Wise Copy Constructor Deep Coping
    BestPractice::BestPractice(const BestPractice &source)
        : BestPractice{*source.m_node_handle}
    {
        ROS_INFO("[%s] Copy Constructor is activated (DEEP COPING) .", __APP_NAME__);
    }

    BestPractice::~BestPractice()
    {
        // dealocate memory
        delete m_point_cloud;
        delete m_node_handle;

        --m_number_objects;

        ROS_INFO("[%s]  Destructor is activated. ", __APP_NAME__);

        ros::shutdown();
    }

    inline bool BestPractice::readParameters()
    {
        if (!m_node_handle->getParam("subscriber_topic", m_subscriber_topic))
        {
            ROS_ERROR("[%s] Could not read parameter 'subscriber_topic'.", __APP_NAME__);
            return false;
        }
        if (!m_node_handle->getParam("publisher_topic", m_publisher_topic))
        {
            ROS_ERROR("[%s] Could not read parameter 'publisher_topic'.", __APP_NAME__);
            return false;
        }
        if (!m_node_handle->getParam("frame_id", m_frame_id))
        {
            ROS_ERROR("[%s] Could not read parameter 'frame_id'.", __APP_NAME__);
            return false;
        }
        return true;
    }

    inline void BestPractice::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &t_point_cloud)
    {
        m_is_cloud_received = true;
        *m_point_cloud = *t_point_cloud;
        std::cout << "m_point_cloud.header.frame_id: " << m_point_cloud->header << std::endl;
    }

    inline void BestPractice::contiuousPointCloudCallback()
    {

        //! do your algorithm here ...with m_point_cloud
        std::cout << "m_point_cloud.header.frame_id: " << m_point_cloud->header << std::endl;
    }

    inline size_t BestPractice::CounterObject()
    {
        return m_number_objects;
    }

    inline void BestPractice::DisplayActiveObjects() const
    {
        ROS_INFO("[%s] Number of active objects: %d", __APP_NAME__, m_number_objects);
    }

}