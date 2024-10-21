#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include "CleaningPathPlanner.h"
#include "tf/tf.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


#include <boost/uuid/uuid.hpp>             // for uuid class
#include <boost/uuid/uuid_generators.hpp>  // for random_generator
#include <boost/uuid/uuid_io.hpp>          // for to_string
#include <experimental/filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

std::vector<geometry_msgs::PoseStamped> extractFirstAndLast(const std::vector<geometry_msgs::PoseStamped>& fullCoverPath) {

    std::vector<geometry_msgs::PoseStamped> final_path;

    if (fullCoverPath.empty()) {
        return final_path;
    }

    int n = fullCoverPath.size();
    int start = 0;

    double episilon = 0.01;

    while (start < n) {
        int end = start;

        // bool flag = (std::abs(fullCoverPath[end].pose.orientation.w - fullCoverPath[start].pose.orientation.w) < episilon
        //           && std::abs(fullCoverPath[end].pose.orientation.x - fullCoverPath[start].pose.orientation.x) < episilon
        //           && std::abs(fullCoverPath[end].pose.orientation.y - fullCoverPath[start].pose.orientation.y) < episilon
        //           && std::abs(fullCoverPath[end].pose.orientation.z - fullCoverPath[start].pose.orientation.z) < episilon);

        // bool flag = (fullCoverPath[end].pose.orientation == fullCoverPath[start].pose.orientation);

        while (end < n && fullCoverPath[end].pose.orientation == fullCoverPath[start].pose.orientation) {
            ++end;
        }

        // final_path.push_back(fullCoverPath[start]);
        // if (end - start > 1) {
        //     final_path.push_back(fullCoverPath[end - 1]);
        // }
        
        final_path.push_back(fullCoverPath[end - 1]);

        start = end;
    }
    return final_path;
}


void write_json(std::vector<geometry_msgs::PoseStamped> final_path){

    std::string block_json_path;

    bool res = ros::param::get("block_json_path", block_json_path);

    if(!res){
        return ;
    }

    bool json_exited = std::experimental::filesystem::exists(block_json_path);

    json j_out;

    boost::uuids::random_generator generator;

    if(json_exited){

        std::ifstream f(block_json_path);
        j_out = json::parse(f);

    }else{

        j_out["block_list"]={};

        std::map<std::string, std::vector<float>> all_places_data;
        std::vector<std::string> all_places;
        if (ros::param::get("all_places", all_places)) {
            for(std::string place_name: all_places){
                std::vector<float> places_data;
                if (ros::param::get(place_name, places_data)) {
                    all_places_data[place_name] = places_data;  // 存储数据
                } else {
                    ROS_ERROR("Failed to get param '%s'", place_name.c_str());
                }
            }
        }
/*----------------------block-----------------------*/
        for(auto i : all_places_data){
            boost::uuids::uuid uuid1 = generator();
            std::string uuid_str1 = boost::uuids::to_string(uuid1);
            boost::uuids::uuid uuid2 = generator();
            std::string uuid_str2 = boost::uuids::to_string(uuid2);
            json j_temp;
            j_temp["name"] = i.first;
            j_temp["uuid"]=uuid_str1;
			j_temp["line_color"]="#eeeeee";
            j_temp["pose_list"]={};
            int cnt = 0;
            for(int j = 0; j < i.second.size(); j+=3){
                    json j_point;
                    j_point["uuid"]=uuid_str2;
                    j_point["pose_type"] = 0;
                    j_point["sn"] = std::to_string(cnt);
                    cnt++;
                    j_point["position"]["pos_x"]=i.second[j];
                    j_point["position"]["pos_y"]=i.second[j+1];
                    j_point["position"]["pos_z"]=i.second[j+2];
                    j_point["orientation"]["ori_w"]=0;
                    j_point["orientation"]["ori_x"]=0;
                    j_point["orientation"]["ori_y"]=0;
                    j_point["orientation"]["ori_z"]=0;
                    j_temp["pose_list"].push_back(j_point);
            }
            j_out["block_list"].push_back(j_temp);
        }
    }


/*----------------------final_path-----------------------*/
    if(!final_path.empty()){
        std::string selected_place;
        bool res_selected = ros::param::get("selected_place", selected_place);
        if(res_selected){
            // boost::uuids::uuid uuid1 = generator();
            // std::string uuid_str1 = boost::uuids::to_string(uuid1);
            boost::uuids::uuid uuid2 = generator();
            std::string uuid_str2 = boost::uuids::to_string(uuid2);

            for (auto& block : j_out["block_list"]) {
                if(block["name"]==selected_place){
                    auto& pose_list = block["pose_list"];
                    pose_list.erase(std::remove_if(pose_list.begin(), pose_list.end(),
                                                [](const json& pose) {
                                                return pose["pose_type"] == 1;
                                                }), 
                                    pose_list.end());
                }
            }

            
            for (auto& block : j_out["block_list"]) {

                if(block["name"]==selected_place){

                    auto& pose_list = block["pose_list"];
                    int cnt = 0;
                    for(auto j : final_path){
                        json j_point;
                        j_point["uuid"]=uuid_str2;
                        j_point["pose_type"] = 1;
                        j_point["sn"] = std::to_string(cnt);
                        cnt++;
                        j_point["position"]["pos_x"]=j.pose.position.x;
                        j_point["position"]["pos_y"]=j.pose.position.y;
                        j_point["position"]["pos_z"]=j.pose.position.z;
                        j_point["orientation"]["ori_w"]=j.pose.orientation.w;
                        j_point["orientation"]["ori_x"]=j.pose.orientation.x;
                        j_point["orientation"]["ori_y"]=j.pose.orientation.y;
                        j_point["orientation"]["ori_z"]=j.pose.orientation.z;
                        pose_list.push_back(j_point);
                    }
                }
            }
        } 
    }

    std::ofstream o(block_json_path);
    o << std::setw(4) << j_out << std::endl;

}



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "write_using_json");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal nextGoal;

  //load the global path.
  tf2_ros::Buffer tf(ros::Duration(10));

  tf2_ros::TransformListener tf2_listener(tf);

  costmap_2d::Costmap2DROS lcr("cleaning_costmap", tf);
  CleaningPathPlanning *pathPlanner = new CleaningPathPlanning(&lcr);

  //full coverage path.
  std::vector<geometry_msgs::PoseStamped> fullCoverPath = pathPlanner->GetPathInROS();

  std::vector<geometry_msgs::PoseStamped> final_path = extractFirstAndLast(fullCoverPath);

  write_json(fullCoverPath);

//   fullCoverPath = final_path;

  int beginNum = fullCoverPath.size();

  //border tracing path.
//  std::vector<geometry_msgs::PoseStamped> borderTrackingPath = pathPlanner->GetBorderTrackingPathInROS();
//  for(int i = 0;i<borderTrackingPath.size();i++)
//  {
//      fullCoverPath.push_back(borderTrackingPath[i]);
//  }

  int time_interval;
  ros::param::param("time_interval", time_interval, 10);

  bool runing_mode;
  ros::param::param("runing_mode",runing_mode, false);

  std::cout<<"runing_mode = "<<runing_mode<<endl;
  std::cout<<"fullcoverpath_size"<<fullCoverPath.size()<<std::endl;

  //main loop
  ros::Rate r(10);
  for(int i = 0; i < fullCoverPath.size(); i++)
  {
      nextGoal.target_pose.header.frame_id = "map";
      nextGoal.target_pose.header.stamp = ros::Time::now();

      geometry_msgs::PoseStamped posestamped = fullCoverPath[i];

      //call move base to plan a long distance.
      nextGoal.target_pose.pose.position.x = posestamped.pose.position.x;
      nextGoal.target_pose.pose.position.y = posestamped.pose.position.y;
      nextGoal.target_pose.pose.position.z = 0;
      nextGoal.target_pose.pose.orientation.w = posestamped.pose.orientation.w;
      nextGoal.target_pose.pose.orientation.x = posestamped.pose.orientation.x;
      nextGoal.target_pose.pose.orientation.y = posestamped.pose.orientation.y;
      nextGoal.target_pose.pose.orientation.z = posestamped.pose.orientation.z;

      ROS_INFO("Sending next goal!");
      ac.sendGoal(nextGoal);
    //   ac.waitForResult(ros::Duration(1));

     if(i==0 && runing_mode ){
        ac.waitForResult(ros::Duration(300));
      }else{

        ac.waitForResult(ros::Duration(time_interval));

      }



      double w,x,y,z;
      w =  posestamped.pose.orientation.w;
      x =  posestamped.pose.orientation.x;
      y =  posestamped.pose.orientation.y;
      z =  posestamped.pose.orientation.z;

      ROS_ERROR("w = %f, x = %f, y = %f, z = %f", w, x, y, z);
      
  
      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          ROS_INFO("Hooray, the base moved a point forward in full path!");
          pathPlanner->SetCoveredGrid(posestamped.pose.position.x,posestamped.pose.position.y);
          pathPlanner->PublishGrid();
          
      }
      else
      {
          ROS_INFO("The base failed to move forward to the next path for some reason!");
          // continue;
      }

      
      pathPlanner->PublishCoveragePath();
      ros::spinOnce();
      r.sleep();



    }

  delete pathPlanner;
  return 0;
}

/*
void reconfigureGlobalPath()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "/move_base";
    double_param.value = pitch;
    conf.doubles.push_back(double_param);

    double_param.name = "kurtana_roll_joint";
    double_param.value = yaw;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}

void reconfigureLocalPath()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "kurtana_pitch_joint";
    double_param.value = pitch;
    conf.doubles.push_back(double_param);

    double_param.name = "kurtana_roll_joint";
    double_param.value = yaw;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}

void reconfigureBorderTracking()
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "kurtana_pitch_joint";
    double_param.value = pitch;
    conf.doubles.push_back(double_param);

    double_param.name = "kurtana_roll_joint";
    double_param.value = yaw;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);
}
*/
