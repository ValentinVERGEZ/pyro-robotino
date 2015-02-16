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
  WorldPluginTutorial() : ModelPlugin(), vel_x(0), vel_y(0), vel_z(0), vel_u(0), vel_v(0), vel_w(0), displayed(false)
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
    	static int cpt = 0;

		// // Apply a small linear velocity to the model.
		// this->model->SetLinearVel(math::Vector3(.03, 0, 0));

    	// // Version ROBOTINO ROUE
    	// if(this->model->GetJoint("wheel0_joint") != 0)
    	// {
    	// 	#define SIN_PI_3	0.866
    	// 	#define COS_PI_3	0.5
    	// 	#define R_WHEEL		0.004

     //        double vx = vel_x*5;
     //        double vy = vel_y*5;
     //        double vw = vel_z*20;

     //        double v1 = vx*SIN_PI_3 - vy*COS_PI_3 + vw*R_WHEEL/3;
     //        double v2 = -vx*SIN_PI_3 - vy*COS_PI_3 + vw*R_WHEEL/3;
     //        double v0 = vy + vw*R_WHEEL/3;

     //        double w1 = v1/R_WHEEL;
     //        double w2 = v2/R_WHEEL;
     //        double w0 = v0/R_WHEEL;

	    // 	this->model->GetJoint("wheel0_joint")->SetMaxForce(2,1);
	    // 	this->model->GetJoint("wheel1_joint")->SetMaxForce(2,1);
	    // 	this->model->GetJoint("wheel2_joint")->SetMaxForce(2,1);
	    // 	this->model->GetJoint("wheel0_joint")->SetVelocity(2,w0);
	    // 	this->model->GetJoint("wheel1_joint")->SetVelocity(2,w1);
	    // 	this->model->GetJoint("wheel2_joint")->SetVelocity(2,w2);
     //        // ROS_INFO_STREAM("Vel " << vx << " " << vy << " " << vw);
     //        // ROS_INFO_STREAM("VelR " << v1 << " " << v2 << " " << v0);
     //        if(cpt++ > 100)
     //        {
     //        	cpt = 0;
	    //         ROS_INFO_STREAM("Wheel 0 " << this->model->GetJoint("wheel0_joint")->GetVelocity(0) << " " <<
	    //         this->model->GetJoint("wheel0_joint")->GetVelocity(1) << " " <<
	    //         this->model->GetJoint("wheel0_joint")->GetVelocity(2) << " " <<
	    //         this->model->GetJoint("wheel0_joint")->GetVelocity(3));
	    //         ROS_INFO_STREAM("Wheel 1 " << this->model->GetJoint("wheel1_joint")->GetVelocity(0) << " " <<
	    //         this->model->GetJoint("wheel1_joint")->GetVelocity(1) << " " <<
	    //         this->model->GetJoint("wheel1_joint")->GetVelocity(2) << " " <<
	    //         this->model->GetJoint("wheel1_joint")->GetVelocity(3));
	    //         ROS_INFO_STREAM("Wheel 2 " << this->model->GetJoint("wheel2_joint")->GetVelocity(0) << " " <<
	    //         this->model->GetJoint("wheel2_joint")->GetVelocity(1) << " " <<
	    //         this->model->GetJoint("wheel2_joint")->GetVelocity(2) << " " <<
	    //         this->model->GetJoint("wheel2_joint")->GetVelocity(3));
	    //         ROS_INFO_STREAM("W " << w1 << " " << w2 << " " << w0 << "\n\n\n\n\n\n\n\n");            	
     //        }
    	// }
    	// else
    	// {
    	// 	ROS_WARN_STREAM("Joint not found");
    	// }

    	// Version ROBOTINO PLANAR
    	double w = this->model->GetLink("base_link")->GetWorldCoGPose().rot.GetYaw();
        double vx = vel_x * cos(w) - vel_y * sin(w);
        double vy = vel_x * sin(w) + vel_y * cos(w);
        double vw = vel_z;

        #define SIN_PI_3	0.866
    	#define COS_PI_3	0.5
    	#define R_WHEEL		0.04
    	#define R_BASE		0.17

        double v1 = vel_x*SIN_PI_3 - vel_y*COS_PI_3 - vel_z*R_BASE/3;
        double v2 = -vel_x*SIN_PI_3 - vel_y*COS_PI_3 - vel_z*R_BASE/3;
        double v0 = vel_y - vel_z*R_BASE/3;

        double w1 = v1/R_WHEEL;
        double w2 = v2/R_WHEEL;
        double w0 = v0/R_WHEEL;

		if(this->model->GetJoint("wheel0_joint") != 0)
		{
	    	this->model->GetJoint("wheel0_joint")->SetMaxForce(0,10);
	    	this->model->GetJoint("wheel0_joint")->SetVelocity(0,w0);			
		}
		if(this->model->GetJoint("wheel1_joint") != 0)
		{
	    	this->model->GetJoint("wheel1_joint")->SetMaxForce(0,10);
	    	this->model->GetJoint("wheel1_joint")->SetVelocity(0,w1);			
		}
		if(this->model->GetJoint("wheel2_joint") != 0)
		{
	    	this->model->GetJoint("wheel2_joint")->SetMaxForce(0,10);
	    	this->model->GetJoint("wheel2_joint")->SetVelocity(0,w2);			
		}

        if(++cpt > 100)
        {
        	cpt = 0;
            ROS_INFO_STREAM("Vx " << this->model->GetJoint("joint_x")->GetVelocity(0));
            ROS_INFO_STREAM("Vy " << this->model->GetJoint("joint_y")->GetVelocity(0));
            ROS_INFO_STREAM("Rotation " << this->model->GetJoint("revolute_w")->GetVelocity(0));
            ROS_INFO_STREAM("V" << vx << " " << vy << " " << vw << "\n\n\n\n\n\n\n\n");            	
        }

    	if(this->model->GetJoint("revolute_w") != 0)
    	{
	    	this->model->GetJoint("revolute_w")->SetMaxForce(0,10);
	    	this->model->GetJoint("revolute_w")->SetVelocity(0,vw);
	    	if(cpt >= 100)
            ROS_INFO_STREAM("Revolute_W " << this->model->GetJoint("revolute_w")->GetForce(0));
    	}
    	if(this->model->GetJoint("joint_x") != 0)
    	{

	    	this->model->GetJoint("joint_x")->SetMaxForce(0,10);
	    	this->model->GetJoint("joint_x")->SetVelocity(0,vx);
	    	if(cpt >= 100)
            ROS_INFO_STREAM("Joint_x " << this->model->GetJoint("joint_x")->GetForce(0));
    	}
    	if(this->model->GetJoint("joint_y") != 0)
    	{

	    	this->model->GetJoint("joint_y")->SetMaxForce(0,10);
	    	this->model->GetJoint("joint_y")->SetVelocity(0,vy);
	    	if(cpt >= 100)
            ROS_INFO_STREAM("Joint_y " << this->model->GetJoint("joint_y")->GetForce(0));
    	}

    	if(this->model->GetJoint("joint_x") == 0 
    		|| this->model->GetJoint("joint_y") == 0
    		|| this->model->GetJoint("revolute_w") == 0)
    	{
    		if(!displayed)
    		{
    			displayed = true;
	    		std::string joints_name("");
	    		if(this->model->GetJoint("joint_x") == 0)
	    		{
	    			joints_name += "Joint_x ";
	    		}
	    		if(this->model->GetJoint("joint_y") == 0)
	    		{
	    			joints_name += "Joint_y ";
	    		}
	    		if(this->model->GetJoint("revolute_w") == 0)
	    		{
	    			joints_name +="Revolute_w ";
	    		}
	    		ROS_WARN_STREAM("Joint " << joints_name <<"not found");
    		}
    	}

    	// // Version TEST
     //    if(this->model->GetJoint("joint_x") != 0)
     //    {
     //        this->model->GetJoint("joint_x")->SetMaxForce(2,10);
     //        this->model->GetJoint("joint_y")->SetMaxForce(2,10);
     //        this->model->GetJoint("joint_w")->SetMaxForce(2,10);
     //        this->model->GetJoint("joint_x")->SetVelocity(2,vel_x*5);
     //        this->model->GetJoint("joint_y")->SetVelocity(2,vel_y*5);
     //        this->model->GetJoint("joint_w")->SetVelocity(2,vel_z*5);
     //        // this->model->SetLinearVel(math::Vector3(1, 1, 1));
     //        ROS_INFO_STREAM("Vel " << vel_x << " " << vel_y << " " << vel_z << " " << vel_u << " " << vel_v << " " << vel_w);          
     //    }
     //    else
     //    {
     //        ROS_WARN_STREAM("Joint not found");
     //    }

    }

private:
	ros::NodeHandle nh_;
	ros::Subscriber cmd_vel_sub_;
	double vel_x;
    double vel_y;
    double vel_z;
    double vel_u;
    double vel_v;
    double vel_w;
    bool displayed;

	void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg)
	{
		vel_x = msg->linear.x;
        vel_y = msg->linear.y;
        vel_z = msg->angular.z;
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