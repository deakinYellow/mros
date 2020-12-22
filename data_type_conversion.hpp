#ifndef DATA_TYPE_CONVERSION_HPP
#define DATA_TYPE_CONVERSION_HPP

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Imu.h>

namespace mros {

namespace data_type_conversion {

    static geometry_msgs::Point EigenPointToGeometryMsgPoint( const Eigen::Vector3d& point ){
        geometry_msgs::Point ret_point;
        ret_point.x = point.x();
        ret_point.y = point.y();
        ret_point.z = point.z();
        return ret_point;
    }

    static Eigen::Vector3d   GeometryMsgPointToEigenPoint( const geometry_msgs::Point& point ){
        Eigen::Vector3d ret_point;
        ret_point.x() = point.x;
        ret_point.y() = point.y;
        ret_point.z() = point.z;
        return ret_point;
    }

    static geometry_msgs::Pose EigenPoseToGeometryMsgPose( const Eigen::Vector3d& position,
                                                    const Eigen::Quaterniond orientation ) {
        geometry_msgs::Pose  pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.w = orientation.w();
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        return pose;
    }

    static geometry_msgs::Quaternion EigenQuaterniondToGeometryMsgQuaternion( const Eigen::Quaterniond& q ){
        geometry_msgs::Quaternion  ret_q;
        ret_q.w = q.w();
        ret_q.x = q.x();
        ret_q.y = q.y();
        ret_q.z = q.z();
        return  ret_q;
    }

    static Eigen::Quaterniond GeometryMsgQuaternionToEigenQuaterniond( const geometry_msgs::Quaternion& q ){
        Eigen::Quaterniond  ret_q;
        ret_q.w() = q.w;
        ret_q.x() = q.x;
        ret_q.y() = q.y;
        ret_q.z() = q.z;
        return  ret_q;
    }

    static geometry_msgs::Vector3 EigenVector3dToGeometryMsgVector3( const Eigen::Vector3d& vector ){
        geometry_msgs::Vector3  ret_vector;
        ret_vector.x = vector.x();
        ret_vector.y = vector.y();
        ret_vector.z = vector.z();
        return  ret_vector;
    }

    static Eigen::Vector3d GeometryMsgVector3ToEigenVectord( const geometry_msgs::Vector3& vector ){
        Eigen::Vector3d  ret_vector;
        ret_vector.x() = vector.x;
        ret_vector.y() = vector.y;
        ret_vector.z() = vector.z;
        return  ret_vector;
    }

    ///std::vector<double> to Eigen::Vector3d
    static Eigen::Vector3d StdVector3dToEigenVector3d( const std::vector<double>& vector ){
        Eigen::Vector3d  ret_vector;
        ret_vector.x() = vector[0];
        ret_vector.y() = vector[1];
        ret_vector.z() = vector[2];
        return  ret_vector;
    }


}  //end of namespace data_type_conversion

} // end of namespace mros

#endif // DATA_TYPE_CONVERSION_HPP
