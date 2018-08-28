/*
 *      Author: Amir Rasouli
 *      email: aras@eecs.yorku.ca
 *
 *      This Code is a ROS package for Gazebo world plugin.
 *      This is designed for manipulating (adding, removing, moving) 3D models in the world environment.
 *      Use: the plugin, compile it and include the path to the library file inside the Gazebo ".world" file.
 *      Dependencies: This code uses ROS services to communicate with Gazebo. It depends on 4 custom messages
 *      (EntityPose, EntityList, GetEntitiesNames, GetEntityPose) all of which are provided in cutom_message package.
 */


#ifndef _WD_PLUGIN_HH_
#define _WDLUGIN_HH_
#include "wd_plugin.hh"

using namespace gazebo;
using namespace std;
GZ_REGISTER_WORLD_PLUGIN(WDPlugin);

WDPlugin::WDPlugin() {}
WDPlugin::~WDPlugin()
{
	this->rosNode->shutdown();
	rosNode.reset();
}

void WDPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

	//Store the model pointer for convenience.
	this->world = _world;
	this->sdf = _sdf;
	namespace_ =  this->world->GetName();
	this->GetParams();
	this->ROSNodeInit();
	this->InitRosTopics();
}
// A template function for reading/ setting default sdf parameters
template<typename T> void WDPlugin::GetSDFParam(T &var, std::string param, T defaultVal)
{

	if (this->sdf->HasElement(param)) {
		var = this->sdf->Get<T>(param);
		gzmsg << param <<" is set to \"" << var << "\" \n";
	}else
	{
		var = defaultVal;
		gzmsg << param <<" is not set, the default value \"" << var << "\" is used\n";
	}
}
void WDPlugin::QueueThread() {
	static const double timeout = 0.01;
	while (this->rosNode->ok()) {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
		ros::spinOnce();
	}
}
// Reads the topic names for subscribers and services
void WDPlugin::GetParams()
{

	//******* Subscribers ********
	this->GetSDFParam<std::string>(this->setEntityPoseTopic,"setEntityPoseTopic","/" + this->namespace_+ "/set_entity_pose");
	this->GetSDFParam<std::string>(this->removeModelTopic,"removeModelTopic","/" + this->namespace_+ "/remove_model");
	this->GetSDFParam<std::string>(this->insertModelTopic,"insertModelTopic","/" + this->namespace_+ "/insert_model");
	this->GetSDFParam<std::string>(this->clearWorldTopic,"clearWorldTopic","/" + this->namespace_+ "/clear_world");
	//******* Services ********
	this->GetSDFParam<std::string>(this->getEntitiesListService,"getEntitiesListService","/" + this->namespace_+ "/get_entities_names");
	this->GetSDFParam<std::string>(this->getEntityPoseService,"getEntityPoseService","/" + this->namespace_+ "/get_entity_pose");

}
//Sets up ROS nodes
void WDPlugin::ROSNodeInit()
{

	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, namespace_,
				ros::init_options::NoSigintHandler);
	}
	this->rosNode.reset(new ros::NodeHandle(namespace_));
}
// Setting up ROS topics
void WDPlugin::InitRosTopics()
{
	//***** subscribers ******
	//Set Entity Pose
	ros::SubscribeOptions sep =
			ros::SubscribeOptions::create <custom_msg::EntityPose>
	(this->setEntityPoseTopic, 100, boost::bind(
			&WDPlugin::OnSetEntityPose, this, _1), ros::VoidPtr(), &this->rosQueue);
	this->setEntityPoseSub = this->rosNode->subscribe(sep);

	//Remove Model
	ros::SubscribeOptions rm =
			ros::SubscribeOptions::create <std_msgs::String>
	(this->removeModelTopic, 100, boost::bind(
			&WDPlugin::OnRemoveModel, this, _1), ros::VoidPtr(), &this->rosQueue);
	this->removeModelSub = this->rosNode->subscribe(rm);

	//Insert Model
	ros::SubscribeOptions im =
			ros::SubscribeOptions::create <custom_msg::EntityPose>
	(this->insertModelTopic, 100, boost::bind(
			&WDPlugin::OnInsertModel, this, _1), ros::VoidPtr(), &this->rosQueue);
	this->insertModelSub = this->rosNode->subscribe(im);

	//clear (reset) world
	ros::SubscribeOptions cw =
				ros::SubscribeOptions::create <std_msgs::Bool>
		(this->clearWorldTopic, 100, boost::bind(
				&WDPlugin::OnClearWorld, this, _1), ros::VoidPtr(), &this->rosQueue);

	this->clearWorldSub = this->rosNode->subscribe(cw);
	//***** Services ******
	this->getEntitiesListSrv = this->rosNode->advertiseService(this->getEntitiesListService , &WDPlugin::GetEntitiesNames, this);
	this->getEntityPoseSrv = this->rosNode->advertiseService(this->getEntityPoseService , &WDPlugin::GetEntityPose, this);
	this->callbackQueueThread = boost::thread(std::bind(&WDPlugin::QueueThread, this));
	ros::spinOnce();
}

//Sends the list of all available models in the world environment
std::vector<std::string> WDPlugin::GetModelsNames()
{
	models = this->world->GetModels();
	std::cout << "models names and indecies: {";
	for ( int i = 0; i < models.size(); i++)
	{
		std::cout << i << "-" << models[i]->GetName() << " ,";
		this->modelsNames.push_back(models[i]->GetName());
	}
	std::cout << "}\n";

	return this->modelsNames;
}
//Sends the list of all available lights in the world environment
std::vector<std::string> WDPlugin::GetLightsNames()
{
	lights = this->world->Lights();
	std::cout << "Lights names and indecies: {";
	for ( int i = 0; i < lights.size(); i++)
	{
		std::cout << i << "-" << lights[i]->GetName() << " ,";
		this->lightsNames.push_back(lights[i]->GetName());
	}
	std::cout << "}\n";

	return this->lightsNames;
}
//Gets the pose of a particular model
math::Pose WDPlugin::GetModelPose(std::string name)
{
	math::Pose pose = this->world->GetModel(name)->GetWorldPose();
	std::cout << "Model " << name << " pose is [" <<
			pose.pos.x   << ", " << pose.pos.y << ", " << pose.pos.z <<", "<<
			pose.rot.GetPitch()<< ", " << pose.rot.GetRoll() << ", " << pose.rot.GetYaw()<< "] \n";

	return pose;
}
//Gets the pose of a particular light
math::Pose WDPlugin::GetLightPose(std::string name)
{
	math::Pose pose = this->world->Light(name)->GetWorldPose();
	std::cout << "Light " << name << " pose is [" <<
			pose.pos.x   << ", " << pose.pos.y << ", " << pose.pos.z <<", "<<
			pose.rot.GetPitch()<< ", " << pose.rot.GetRoll() << ", " << pose.rot.GetYaw()<< "] \n";
	return pose;
}
//Sets the pose of a particular model
void WDPlugin::SetModelPose(std::string name, math::Pose pose)
{
	this->world->GetModel(name)->SetWorldPose(pose);
	std::cout << "Model " << name << " new pose is [" <<
			pose.pos.x   << ", " << pose.pos.y << ", " << pose.pos.z <<", "<<
			pose.rot.GetPitch()<< ", " << pose.rot.GetRoll() << ", " << pose.rot.GetYaw()<< "] \n";

}
//Sets the pose of a particular light
void WDPlugin::SetLightPose(std::string name, math::Pose pose)
{
	this->world->Light(name)->SetWorldPose(pose);
	std::cout << "Model " << name << " new pose is [" <<
			pose.pos.x   << ", " << pose.pos.y << ", " << pose.pos.z <<", "<<
			pose.rot.GetPitch()<< ", " << pose.rot.GetRoll() << ", " << pose.rot.GetYaw()<< "] \n";
}
//Removes a particular model from the world environment
void WDPlugin::RemoveModel(std::string name)
{
	if (this->world->GetModel(name) != nullptr)
	{
		this->world->RemoveModel(name);
	}else{
		ROS_ERROR("Model %s does not exist!", name.c_str());
	}
}
//Inserts a model in the world environment
void WDPlugin::InsertModel(std::string object,  math::Pose pose, std::string name)
{
	sdf::ElementPtr uri(new sdf::Element);
	sdf::ElementPtr modelPose(new sdf::Element);
	sdf::ElementPtr incl(new sdf::Element);

	uri->SetName("uri");
	uri->AddValue("string", "model://"+object, 1);
	modelPose->SetName("pose");
	std::string pse = to_string(pose.pos.x)+" "+to_string(pose.pos.y)+" "+to_string(pose.pos.z)
					+" "+to_string(pose.rot.GetRoll())+" "+to_string(pose.rot.GetPitch())+" "+to_string(pose.rot.GetYaw());
	modelPose->AddValue("Pose", pse, 0);
	incl->SetName("include");
	incl->InsertElement(modelPose);
	incl->InsertElement(uri);
	sdf::SDF modInput;

	if (name == "")
	{
		modInput.Root(incl);
	}else{
		sdf::ElementPtr modelName(new sdf::Element);
		modelName->SetName("model");
		modelName->AddAttribute("name","string", name, 1);
		modelName->InsertElement(incl);
		modInput.Root(modelName);

	}
	this->world->InsertModelSDF(modInput);

//	 sdf::SDF sphereSDF;
//	    sphereSDF.SetFromString(
//	       "<sdf version ='1.4'>\
//	         <model name ='sphere'>\
//	           <pose>1 2 0 0 0 0</pose>\
//	        <link name ='link'>\
//	          <pose>0 0 .5 0 0 0</pose>\
//	          <collision name ='collision'>\
//	            <geometry>\
//	              <sphere><radius>0.5</radius></sphere>\
//	            </geometry>\
//	          </collision>\
//	          <visual name ='visual'>\
//	            <geometry>\
//	              <sphere><radius>0.5</radius></sphere>\
//	            </geometry>\
//	          </visual>\
//	        </link>\
//	          </model>\
//	        </sdf>");
}

//services
bool WDPlugin::GetEntitiesNames(custom_msg::GetEntitiesNames::Request& req,
		custom_msg::GetEntitiesNames::Response &res)
{

	if (req.entity_type == "Light" || req.entity_type == "light")
	{
		res.entities_list = this->GetLightsNames();
		return true;
	}else if (req.entity_type == "Model" || req.entity_type == "model"){
		res.entities_list = this->GetModelsNames();
		return true;
	}else{
		ROS_INFO("Wrong type of entities is specified!");
		return false;
	}


}
bool WDPlugin::GetEntityPose(custom_msg::GetEntityPose::Request& req,
		custom_msg::GetEntityPose::Response& res)
{
	math::Pose pose;
	if (req.entity_type == "Light" || req.entity_type == "light")
	{
		pose = GetLightPose(req.entity_name);

	}else if (req.entity_type == "Model" || req.entity_type == "model"){
		pose = GetModelPose(req.entity_name);
	}else{
		ROS_INFO("Wrong type of entities is specified!");
		return false;
	}

	geometry_msgs::Pose rosPose;
	rosPose.position.x = pose.pos.x;
	rosPose.position.y = pose.pos.y;
	rosPose.position.z = pose.pos.z;
	rosPose.orientation.x = pose.rot.x;
	rosPose.orientation.y = pose.rot.y;
	rosPose.orientation.z = pose.rot.z;
	rosPose.orientation.w = pose.rot.w;
	res.pose = rosPose;
	return true;
}


//call back functions
void WDPlugin::OnSetEntityPose(const custom_msg::EntityPose::ConstPtr &msg)
{
	math::Vector3 vec(msg->pose.position.x, msg->pose.position.y,msg->pose.position.z);
	math::Quaternion qt( msg->pose.orientation.w, msg->pose.orientation.x,msg->pose.orientation.y, msg->pose.orientation.z);
	if (msg->entity_type == "Light" || msg->entity_type == "light")
	{
		SetLightPose(msg->entity_name, math::Pose(vec,qt));
	}else if (msg->entity_type == "Model" || msg->entity_type == "model"){
		SetModelPose(msg->entity_name, math::Pose(vec,qt));
	}else{
		ROS_INFO("Wrong type of entities is specified!");
	}
}
void WDPlugin::OnRemoveModel(const std_msgs::String::ConstPtr &msg)
{
	this->RemoveModel(msg->data);
};
void WDPlugin::OnInsertModel(const custom_msg::EntityPose::ConstPtr &msg)
{
	math::Vector3 vec(msg->pose.position.x, msg->pose.position.y,msg->pose.position.z);
	math::Quaternion qt( msg->pose.orientation.w, msg->pose.orientation.x,msg->pose.orientation.y, msg->pose.orientation.z);
	this->InsertModel(msg->object_name, math::Pose(vec,qt), msg->entity_name);
};
void WDPlugin::OnClearWorld (const std_msgs::Bool::ConstPtr &msg)
{
	if(msg->data)
		this->world->Clear();
}
#endif
