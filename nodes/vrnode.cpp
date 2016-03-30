/*
 Software License Agreement (BSD License)

 Copyright (c) 2012, Scott Niekum
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the Willow Garage nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 author: Scott Niekum, Matt Buckley
*/


#include <sstream>
#include <ar_track_alvar/CvTestbed.h>
#include <ar_track_alvar/AlvarMarker.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
//#include <gazebo/World.hh>
#include <string>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <std_msgs/ColorRGBA.h>
#include <tf/transform_broadcaster.h>
#include <vr_track_alvar/colorify.h>
#include <visualization_msgs/Marker.h>
//#include <ar_track_alvar/ParamsConfig.h>
#include <gazebo_msgs/LinkStates.h>


using namespace alvar;
using namespace std;

ros::Subscriber sim_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar::AlvarMarkers arPoseMarkers_;
visualization_msgs::Marker rvizMarker_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
double marker_size = 5.5;
int markernum = 0;
std::string markers[18];
std::string markerIDs[18];

bool enableSwitched = false;
bool enabled = true;
double max_frequency = 30.0;
std::string output_frame;
string reference_frame = "base_link";
char res[50];
void getCapCallback (const gazebo_msgs::LinkStates & image_msg);

void fromTFAnalyze() {
    //ROS_INFO("Reference exists: %d",(int)tf_listener->frameExists(output_frame));
    if (!tf_listener->frameExists(output_frame)) {
        ROS_WARN("Output frame %s does not exist",output_frame.c_str());
        return;
    }
    for (int iter=0;iter<markernum;++iter) {
        //ROS_INFO("%s exists: %d",markers[iter].c_str(),(int)tf_listener->frameExists(output_frame));
        int ID = std::atoi(markerIDs[iter].c_str());
        std::string tfname = markers[iter];
        std::string num = markerIDs[iter];
        //ros::Time now = ros::Time::now(0);
        if (!tf_listener->frameExists(tfname.c_str())) {
            ROS_WARN("Marker frame %s does not exist",tfname.c_str());
            continue;
        }
        if (!tf_listener->canTransform(output_frame,tfname,ros::Time(0))) {
            //ROS_WARN("No transform from output frame %s to marker frame %s",output_frame.c_str(),tfname.c_str());
            continue;
        }
        tf::StampedTransform marktf;
        tf_listener->lookupTransform(output_frame,tfname,ros::Time(0),marktf);
        std::string markerFrame = "ar_marker_";
        markerFrame += num;
        tf::StampedTransform markerTrans (marktf, marktf.stamp_, output_frame, markerFrame.c_str());
        tf_broadcaster->sendTransform(markerTrans);
        
        tf::poseTFToMsg (marktf, rvizMarker_.pose);
        rvizMarker_.header.frame_id = output_frame;
        rvizMarker_.header.stamp = marktf.stamp_;
        rvizMarker_.id = ID;
        rvizMarker_.scale.x = 1.0 * marker_size/100.0;
        rvizMarker_.scale.y = 1.0 * marker_size/100.0;
        rvizMarker_.scale.z = 0.2 * marker_size/100.0;
        rvizMarker_.ns = "basic_shapes";
        rvizMarker_.type = visualization_msgs::Marker::CUBE;
        rvizMarker_.action = visualization_msgs::Marker::ADD;
        rvizMarker_.color = colormarker(ID);
        rvizMarker_.color.a = 1.0;
        rvizMarker_.lifetime = ros::Duration (1.0);
        rvizMarkerPub_.publish (rvizMarker_);
        
        //Create the pose marker messages
        ar_track_alvar::AlvarMarker ar_pose_marker;
        tf::poseTFToMsg (marktf, ar_pose_marker.pose.pose);
        ar_pose_marker.header.frame_id = output_frame;
        ar_pose_marker.header.stamp = marktf.stamp_;
        ar_pose_marker.pose.header.stamp = marktf.stamp_;
        ar_pose_marker.pose.header.frame_id = output_frame;
        ar_pose_marker.id = ID;
        arPoseMarkers_.markers.push_back (ar_pose_marker);
        
        //rvizMarker_.pose.header.frame_id = output_frame;
        //rvizMarker_.pose.header.stamp = now;
        //rvizMarker_.pose = model_msg.pose[iter];
    }
    arMarkerPub_.publish (arPoseMarkers_);
}

void getCapCallback (const gazebo_msgs::LinkStates & model_msg)
{

    int arrsize = model_msg.name.size();//sizeof(model_msg.name)/sizeof(model_msg.name[0]);
    ROS_INFO("Got stuff of size: %d",arrsize);
    arPoseMarkers_.markers.clear ();
    string strname;
    for (int iter=0;iter<arrsize;++iter) {
        strname = string(model_msg.name[iter]);
        //ROS_INFO(strname.c_str());
        if (strname.length() >= 7) {
            if (strname.substr(0,6)=="marker") { //if it's a marker
                strname = strname.substr(6);
                int id = -1;
                id = std::atoi(strname.c_str());
                if ((id == 0) && (strname[0]!='0')) {
                    continue;
                }
                if (!((id >= 0) && (id <= 17))) {
                    continue;
                }
                std::string markerFrame = "ar_marker_";
                std::stringstream out;
                out << id;
                ros::Time now = ros::Time::now();
                std::string id_string = out.str();
                markerFrame += id_string;
                tf::Transform posetrans;
                poseMsgToTF(model_msg.pose[iter],posetrans);
                tf::StampedTransform simToMarker (posetrans, now, reference_frame, markerFrame.c_str());
                tf_broadcaster->sendTransform(simToMarker);
                tf::StampedTransform simToOutput;
                tf_listener->lookupTransform(output_frame,reference_frame,now,simToOutput);
                //Create the rviz visualization messages
                tf::Transform otherTF;
                tf::poseMsgToTF(model_msg.pose[iter],otherTF);
                tf::Pose outputPose = otherTF*simToOutput;
                tf::poseTFToMsg (outputPose, rvizMarker_.pose);
                //rvizMarker_.pose.header.frame_id = output_frame;
                //rvizMarker_.pose.header.stamp = now;
                //rvizMarker_.pose = model_msg.pose[iter];
                rvizMarker_.header.frame_id = output_frame;
                rvizMarker_.header.stamp = now;
                rvizMarker_.id = id;
                rvizMarker_.scale.x = 1.0 * marker_size/100.0;
                rvizMarker_.scale.y = 1.0 * marker_size/100.0;
                rvizMarker_.scale.z = 0.2 * marker_size/100.0;
                rvizMarker_.ns = "basic_shapes";
                rvizMarker_.type = visualization_msgs::Marker::CUBE;
                rvizMarker_.action = visualization_msgs::Marker::ADD;
                rvizMarker_.color = colormarker(id);
                rvizMarker_.lifetime = ros::Duration (1.0);
                rvizMarkerPub_.publish (rvizMarker_);
                
                //Create the pose marker messages
                ar_track_alvar::AlvarMarker ar_pose_marker;
                tf::poseTFToMsg (outputPose, ar_pose_marker.pose.pose);
                ar_pose_marker.header.frame_id = output_frame;
                ar_pose_marker.header.stamp = now;
                ar_pose_marker.pose.header.stamp = now;
                ar_pose_marker.pose.header.frame_id = output_frame;
                ar_pose_marker.id = id;
                arPoseMarkers_.markers.push_back (ar_pose_marker);	
            }
        }
    }
    arMarkerPub_.publish (arPoseMarkers_);

}

int main(int argc, char *argv[])
{
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n, pn("~");
    ROS_INFO("Init");
	if(argc < 4){
		std::cout << std::endl;
		cout << "Not enough arguments provided." << endl;
		return 0;
	}

	// Get params from command line
    marker_size = atof(argv[1]);
    output_frame = argv[2];
    //max_frequency = argv[3];
    //max_frequency = atof(argv[2]);
	//marker_detector.SetMarkerSize(marker_size);
    
    for (int i=3;i<argc;++i) {
        stringstream stream(argv[i]);
        getline(stream, markers[markernum], ':');
        getline(stream, markerIDs[markernum]);
        ROS_INFO("Loading marker link: %s with ID: %s",markers[markernum].c_str(),markerIDs[markernum].c_str());
        if (markerIDs[markernum].length()==0) {
            //do stuff
        }
        ++markernum;
    }

  // Set dynamically configurable parameters so they don't get replaced by default values


	//cam = new Camera(n, cam_info_topic);
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise < ar_track_alvar::AlvarMarkers > ("ar_pose_marker", 0);
	rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);

	ros::Duration(1.0).sleep();
	ros::spinOnce();	
	

  ros::Rate rate(max_frequency);

  while (ros::ok())
  {
    ros::spinOnce();
    fromTFAnalyze();
    rate.sleep();


  }

    return 0;
}
