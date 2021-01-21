#ifndef TF_HPP
#define TF_HPP

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>


namespace mros {

class TF{

public:
  explicit TF(){
    ;
  }
  //frame_id通常设置为map
  explicit TF( const std::string& frame_id ) : frame_id_( frame_id ){
    ;
  }

  //根据当前3d姿态发布tf
  void SendTransform( const std::string& tf_id,
                      const geometry_msgs::Pose& pose ){
    transform_->setOrigin( tf::Vector3( pose.position.x , pose.position.y, pose.position.z ) );
    tf::Quaternion q;
    q.setW( pose.orientation.w );
    q.setX( pose.orientation.x );
    q.setY( pose.orientation.y );
    q.setZ( pose.orientation.z );
    q.normalize();
    transform_->setRotation( q );
    broadcaster_->sendTransform( tf::StampedTransform( *transform_, ros::Time::now(), frame_id_, tf_id ) );
  }

  //根据当前2d姿态发布tf
  //输入pose.theta范围[-PI,PI]
  void SendTransform( const std::string& tf_id,
                      const geometry_msgs::Pose2D& pose ){

    transform_->setOrigin( tf::Vector3( pose.x , pose.y, 0 ) );
    tf::Quaternion q;
    q.setRPY( 0, 0, pose.theta );
    q.normalize();
    transform_->setRotation( q );
    broadcaster_->sendTransform( tf::StampedTransform( *transform_, ros::Time::now(), frame_id_, tf_id ) );
  }

  ~TF(){
    delete broadcaster_;
    delete transform_;
  }

private:
  //tf广播对象
  tf::TransformBroadcaster* broadcaster_ = new tf::TransformBroadcaster;
  //tf变换对象
  tf::Transform* transform_ = new tf::Transform;
  //参考系ID
  std::string frame_id_="map";

};  //class TF


} //namespace mros

#endif // TF_HPP


