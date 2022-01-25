#include <pegasus_gazebo_plugins/closed_loop_plugin.h>
#include <math.h>
#include <vector>

namespace gazebo


{

ClosedLoopPlugin::ClosedLoopPlugin()
{
  kill_sim = false;

  joint_.reset();
  parent_.reset();
  child_.reset();
}

ClosedLoopPlugin::~ClosedLoopPlugin()
{
  this->updateConnection.reset();

  kill_sim = true;
}

void ClosedLoopPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  ros::NodeHandle model_nh;
  model_ = _parent;
  world_ = model_->GetWorld();
  physics_ = world_->Physics();
  

  // Error message if the model couldn't be found
  if (!model_)
  {
    ROS_ERROR("Parent model is NULL! GazeboNaoqiControlPlugin could not be loaded.");
    return;
  }

  // Check that ROS has been initialized
  if(!ros::isInitialized())
  {
    ROS_ERROR("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
  }

  if(!_sdf->HasElement("joint"))
  {
    ROS_ERROR("No joint element present. ClosedLoopPlugin could not be loaded.");
    return;
  }

  joint_name_ = _sdf->GetElement("joint")->Get<std::string>();


  if(!_sdf->HasElement("child"))
  {
    ROS_ERROR("No child element present. ClosedLoopPlugin could not be loaded.");
    return;
  }

  child_name_ = _sdf->GetElement("child")->Get<std::string>();

  if(!_sdf->HasElement("parent"))
  {
    ROS_ERROR("No parent element present. ClosedLoopPlugin could not be loaded.");
    return;
  }

  parent_name_ = _sdf->GetElement("parent")->Get<std::string>();



  child_ = model_->GetLink(child_name_);
  if(!child_)
  {
    ROS_ERROR("No Link named %s. ClosedLoopPlugin could not be loaded.", child_name_.c_str());
    return;
  }

  parent_ = model_->GetLink(parent_name_);
  if(!parent_)
  {
    ROS_ERROR("No Link named %s. ClosedLoopPlugin could not be loaded.", parent_name_.c_str());
    return;
  }
  //we get the string given into the <rotation> tag
  rotation_ = _sdf->GetElement("rotation")->Get<std::string>();
  //we get the string given into the <position> tag
  position_ = _sdf->GetElement("position")->Get<std::string>();

  //we convert the strings into a string vector, spliting it at each space using the function Split_String
  std::vector<std::string> rotations_splited = Split_String(rotation_);
  std::vector<std::string> positions_splited = Split_String(position_);

  //we convert the splited string vector into floats
  std::vector<float>  rotations_splited_converted = Convert_to_float(rotations_splited);
  std::vector<float>  positions_splited_converted = Convert_to_float(positions_splited);


  model_->CreateJoint(joint_name_,"revolute2",parent_,child_);
  physics::JointPtr j = physics_->CreateJoint("revolute2");
  j->SetName(joint_name_);
  // j->SetLowerLimit(1,0.0);
  // j->SetUpperLimit(1,3.14);
  //math::Pose doesn't work too so we change it to ignition::math::Pose3d
  ignition::math::Pose3d jointOrigin = ignition::math::Pose3d(
                                                                positions_splited_converted[0],
                                                                positions_splited_converted[1],
                                                                positions_splited_converted[2],
                                                                rotations_splited_converted[0],
                                                                rotations_splited_converted[1],
                                                                rotations_splited_converted[2]
                                                              );

  
  j->Load(parent_,child_,jointOrigin);

  j->Attach(parent_,child_);


  
  j->Init();
  //vector3 is changed to vector3d
  ignition::math::Vector3d jointaxis = ignition::math::Vector3d(1,0,0);
  j->SetAxis(1,jointaxis);
  j->Update();

  msgs::Joint jointMsg;

  jointMsg.set_name(joint_name_);

  jointMsg.set_parent(parent_name_);

  jointMsg.set_child(child_name_);

  


    // type
  jointMsg.set_type(gazebo::msgs::ConvertJointType("revolute2"));

  // rendering::JointVisual join_vis_test = rendering::JointVisual();
  // rendering::ScenePtr scene = rendering::get_scene();
  // rendering::VisualPtr joint_ptr = scene->GetVisual(parent_name_);

  // forceVector = rendering_vis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  // for(int k = 0; k < 6; ++k) // -> needs three lines, so 6 points
  //   forceVector->AddPoint(ignition::math::Vector3d(0,0,0));
  // forceVector->setMaterial("Gazebo/Blue");
  // forceVector->setVisibilityFlags(GZ_VISIBILITY_GUI);
  // rendering_vis->SetVisible(true);
  
  

  // forceVector->Update();
  // rendering::VisualPtr vis_ptr = rendering::Visual("joint",joint_ptr,true);
  // rendering::JointVisual joint_ptr2 = rendering::JointVisual("dfs",joint_ptr);

  // joint_ptr2.Load(jointMsg);

  // rendering::ArrowVisual arrow = rendering::ArrowVisual::CreateAxis(jointaxis,parent_name_,jointMsg);

  // parentVis = scene->GetVisual(jointMsg);

  // joint_Visual.JointVisual();

  // arraow_vis.CreateAxis(jointaxis,parent_name_,jointMsg);

  // auto msg = msgs.ConvertJointType(joint_name_) 

  // joint_rendering.JointVisual(joint_name_,rendering_vis);
  
  
  // jointVisual->SetType(ignition::rendering::JointVisualType::JVT_REVOLUTE2);


 
 




  printf("\n__ClosedLoopPlugin Load finished___\n");

}

//function used to split a string at each space into a string vector
std::vector<std::string> ClosedLoopPlugin::Split_String(const std::string& subject)
{


  std::vector<std::string> array;
  std::stringstream ss(subject);
  std::string tmp;
  while(std::getline(ss, tmp, ' '))
  {
    array.push_back(tmp);
  }

  return array;
}

//function used to convert a string vector to a float vector
std::vector<float> ClosedLoopPlugin::Convert_to_float(const std::vector<std::string>& subject)
{

  std::vector<float> results(subject.size());


  for(int i = 0; i<subject.size();i++)
  {

    results[i] = (float)std::atof(subject[i].c_str());


  }

  return results;

}





void ClosedLoopPlugin::UpdateChild()
{
  static ros::Duration period(world_->Physics()->GetMaxStepSize());

}

GZ_REGISTER_MODEL_PLUGIN(ClosedLoopPlugin);

}

