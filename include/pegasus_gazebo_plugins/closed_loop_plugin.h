
#ifndef PEGASUS_GAZEBO_PLUGINS_CLOSED_LOOP_PLUGIN
#define PEGASUS_GAZEBO_PLUGINS_CLOSED_LOOP_PLUGIN
// ROS includes
#include <ros/ros.h>

// ros_control
#include <control_toolbox/pid.h>

// Boost includes
#include <boost/bind.hpp>

// Gazebo includes
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// #include <gazebo/rendering/ogre_gazebo.h>
// #include <gazebo/rendering/DynamicLines.hh>
// #include <gazebo/rendering/Visual.hh>
// #include <gazebo/rendering/JointVisual.hh>
// #include <gazebo/rendering/ArrowVisual.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/rendering/Visual.hh>
#include <math.h>


// #include <gazebo/rendering/UserCamera.hh>
// #include <gazebo/rendering/Scene.hh>
// #include <gazebo/rendering/RenderTypes.hh>

#include <ignition/common/Console.hh>
#include <gazebo/transport/TransportIface.hh>
#include <gazebo/msgs/msgs.hh>

// #include <ignition/rendering/Scene.hh>

// #include <ignition/msgs/Utility.hh>

// #include "ignition/rendering/Capsule.hh>
// #include <ignition/rendering/COMVisual.hh>
// #include <ignition/rendering/Geometry.hh>
// #include <ignition/rendering/Heightmap.hh>
// #include <ignition/rendering/HeightmapDescriptor.hh>
// #include <ignition/rendering/InertiaVisual.hh>
// #include <ignition/rendering/JointVisual.hh>



//
#include <ignition/math/Pose3.hh>
#include <vector>


// #define GAZEBO_VISIBLE


namespace gazebo

{ typedef rendering::DynamicLines* LinePtr;
  class ClosedLoopPlugin : public ModelPlugin
  {
    public:
      ClosedLoopPlugin();
      ~ClosedLoopPlugin();

      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void UpdateChild();
   


    private:
      std::vector<float> Convert_to_float(const std::vector<std::string>& subject);
      std::vector<std::string> Split_String(const std::string& subject);

      // Parameters
      std::string joint_name_, child_name_, parent_name_;
      std::string rotation_;
      std::string position_;

      sdf::ElementPtr insert_sdf_;

      bool kill_sim;

      // Pointers to the joints
      physics::JointPtr joint_;

      // Pointers to the links
      physics::LinkPtr parent_, child_;

      // Pointer to the model
      physics::ModelPtr model_;

      // Pointer to the world
      physics::WorldPtr world_;

      // Pointer to the physics_
      physics::PhysicsEnginePtr physics_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

   

      rendering::JointVisualPtr joint_Visual;

      LinePtr forceVector;

      rendering::VisualPtr  rendering_vis;
      
      // rendering::ArrowVisualPtr arraow_vis;

  


  


  

      
  

  };
}

#endif
