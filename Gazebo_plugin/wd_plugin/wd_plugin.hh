#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <custom_msg/EntityPose.h>
#include <custom_msg/EntityList.h>
#include <custom_msg/GetEntitiesNames.h>
#include <custom_msg/GetEntityPose.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <string>

namespace gazebo {
class  WDPlugin: public WorldPlugin
{
public:
	WDPlugin();
	virtual ~WDPlugin();
private:
	physics::WorldPtr world;
	physics::LightPtr light;
	physics::ModelPtr model;
	std::vector<physics::LightPtr> lights;
	std::vector<physics::ModelPtr> models;
	std::vector<std::string> modelsNames;
	std::vector<std::string> lightsNames;
	sdf::ElementPtr sdf;
	std::unique_ptr<ros::NodeHandle> rosNode;
	ros::Subscriber  setEntityPoseSub, removeModelSub, insertModelSub, clearWorldSub;
	ros::ServiceServer getEntitiesListSrv, getEntityPoseSrv;
	std::string  setEntityPoseTopic, removeModelTopic, insertModelTopic, clearWorldTopic;
	std::string  getEntitiesListService, getEntityPoseService;
	ros::CallbackQueue rosQueue;
	boost::thread callbackQueueThread;
	std::string namespace_;
public:
	void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

private:
template<typename T> void GetSDFParam(T &var, std::string param, T defaultVal);
void QueueThread();
void GetParams();
void ROSNodeInit();
void InitRosTopics();
std::vector<std::string> GetModelsNames();
std::vector<std::string> GetLightsNames();
math::Pose GetModelPose(std::string name);
math::Pose GetLightPose(std::string name);
void SetModelPose(std::string name, math::Pose pose);
void SetLightPose(std::string name, math::Pose pose);
void RemoveModel(std::string name);
void InsertModel(std::string object,  math::Pose pose, std::string name = "");

//Call Back functions
bool GetEntitiesNames(custom_msg::GetEntitiesNames::Request& req, custom_msg::GetEntitiesNames::Response& res);
bool GetEntityPose(custom_msg::GetEntityPose::Request& req, custom_msg::GetEntityPose::Response& res);
void OnSetEntityPose(const custom_msg::EntityPose::ConstPtr &msg);
void OnRemoveModel(const std_msgs::String::ConstPtr &msg);
void OnInsertModel(const custom_msg::EntityPose::ConstPtr &msg);
void OnClearWorld (const std_msgs::Bool::ConstPtr &msg);

};
};

