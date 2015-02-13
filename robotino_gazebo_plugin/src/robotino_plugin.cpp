#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>


#include <geometry_msgs/TwistStamped.h>

namespace gazebo
{
class WorldPluginTutorial : public ModelPlugin
{
public:
  WorldPluginTutorial() : ModelPlugin(), vel_x(0.1), vel_y(0.1), vel_z(0.1)
  {  	
    ROS_WARN("Starting robotino_plugin");
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &WorldPluginTutorial::cmdVelCallback, this);
  }

  ~WorldPluginTutorial()
  {
  	cmd_vel_sub_.shutdown();
  }

  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!");

	// Store the pointer to the model
	this->model = _parent;

	// Listen to the update event. This event is broadcast every
	// simulation iteration.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	  boost::bind(&WorldPluginTutorial::OnUpdate, this, _1));
  }

// Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(math::Vector3(.03, 0, 0));
    	// if(this->model->GetJoint("wheel0_joint") != 0)
    	// {
	    // 	this->model->GetJoint("wheel0_joint")->SetMaxForce(0,10);
	    // 	this->model->GetJoint("wheel1_joint")->SetMaxForce(0,10);
	    // 	this->model->GetJoint("wheel2_joint")->SetMaxForce(0,10);
	    // 	this->model->GetJoint("wheel0_joint")->SetVelocity(0,vel*20);
	    // 	this->model->GetJoint("wheel1_joint")->SetVelocity(0,vel*20);
	    // 	this->model->GetJoint("wheel2_joint")->SetVelocity(0,vel*20);
	    // 	// ROS_INFO_STREAM("Vel " << vel);    		
    	// }
    	// else
    	// {
    	// 	ROS_WARN_STREAM("Joint not found");
    	// }

        if(this->model->GetJoint("joint") != 0)
        {
            vel_x *= 20;
            this->model->GetJoint("joint")->SetForce(0,vel_x);
            this->model->GetJoint("joint")->SetForce(1,vel_y);
            this->model->GetJoint("joint")->SetForce(2,vel_z);
            ROS_INFO_STREAM("Vel " << vel_x << " " << vel_y << " " << vel_z);          
        }
        else
        {
            ROS_WARN_STREAM("Joint not found");
        }

    }

private:
	ros::NodeHandle nh_;
	ros::Subscriber cmd_vel_sub_;
	double vel_x;
    double vel_y;
    double vel_z;

	void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
	{
		vel_x = msg->linear.x;
        vel_y = msg->linear.y;
        vel_z = msg->linear.z;
		// double linear_x = msg->linear.x;
		// double linear_y = msg->linear.y;
		// double angular = msg->angular.z;

		// if ( fabs( linear_x ) > max_linear_vel_ )
		// {
		// 	if( linear_x > 0.0 )
		// 		linear_x = max_linear_vel_;
		// 	else
		// 		linear_x = -max_linear_vel_;
		// }
		// else if( fabs( linear_x ) <  min_linear_vel_ && fabs( linear_x ) > 0.0 )
		// {
		// 	if( linear_x > 0.0 )
		// 		linear_x = min_linear_vel_;
		// 	else
		// 		linear_x = -min_linear_vel_;
		// }

		// if ( fabs( linear_y ) > max_linear_vel_ )
		// {
		// 	if( linear_y > 0.0 )
		// 		linear_y = max_linear_vel_;
		// 	else
		// 		linear_y = -max_linear_vel_;
		// }
		// else if( fabs( linear_y ) <  min_linear_vel_ && fabs( linear_y ) > 0.0 )
		// {
		// 	if( linear_y > 0.0 )
		// 		linear_y = min_linear_vel_;
		// 	else
		// 		linear_y = -min_linear_vel_;
		// }

		// if ( fabs( angular ) > max_angular_vel_ )
		// {
		// 	if( angular > 0.0 )
		// 		angular = max_angular_vel_;
		// 	else
		// 		angular = -max_angular_vel_;
		// }
		// else if( fabs( angular ) <  min_angular_vel_ && fabs( angular ) > 0.0 )
		// {
		// 	if( angular > 0.0 )
		// 		angular = min_angular_vel_;
		// 	else
		// 		angular = -min_angular_vel_;
		// }

		// setVelocity( linear_x, linear_y, angular);
	}

private:
    // Pointer to the model
	physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

};
GZ_REGISTER_MODEL_PLUGIN(WorldPluginTutorial)
}