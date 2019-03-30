/**
@author     johnmper
@file       pupil_nodelet.cc
@brief      Nodelet responsable for creating a bridge between Pupil Capture Software and ROS.
*/

#define DEPTH_MIN_Z 700U
#define DEPTH_MAX_Z 3500U

// STANDARD INCLUDES
#include<iostream>
#include<iomanip>
#include<fstream>
#include<vector>
#include<deque>

// ROS INCLUDES
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include<opencv2/core/core.hpp>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<image_transport/image_transport.h>

// ROS MESSAGES
#include<pupil_msgs/gaze_datum.h>
#include<pupil_msgs/pupil_datum.h>
#include<pupil_msgs/frame.h>
#include<geometry_msgs/TransformStamped.h>
#include<sensor_msgs/fill_image.h>
#include<sensor_msgs/image_encodings.h>
#include<tf/transform_broadcaster.h>
#include<tf2_ros/static_transform_broadcaster.h>

// NETWORK INCLUDES
#include<zmq.hpp>
#include<msgpack.hpp>

// LOCAL INCLUDES
#include<ros_utils/style.hh>
#include<pupil_comms/pupil_adaptor.hh>
#include<nlohmann/json.hpp>


using hr_clock = std::chrono::high_resolution_clock;
#define MS_CAST(a,b) std::chrono::duration_cast<std::chrono::microseconds>(a-b)


//===========================================================================================================
//   #####  ######   ##   #       ####  ###### #    #  ####  ######    #    # ##### # #
//   #    # #       #  #  #      #      #      ##   # #      #         #    #   #   # #
//   #    # #####  #    # #       ####  #####  # #  #  ####  #####     #    #   #   # #
//   #####  #      ###### #           # #      #  # #      # #         #    #   #   # #
//   #   #  #      #    # #      #    # #      #   ## #    # #         #    #   #   # #
//   #    # ###### #    # ######  ####  ###### #    #  ####  ######     ####    #   # ######
//===========================================================================================================
namespace pupil_comms{


    struct sync_header{
        std_msgs::Header header;
        uint64_t index;
    };


//===========================================================================================================
//   ######                         #     #
//   #     # #    # #####  # #      ##    #  ####  #####  ###### #      ###### #####
//   #     # #    # #    # # #      # #   # #    # #    # #      #      #        #
//   ######  #    # #    # # #      #  #  # #    # #    # #####  #      #####    #
//   #       #    # #####  # #      #   # # #    # #    # #      #      #        #
//   #       #    # #      # #      #    ## #    # #    # #      #      #        #
//   #        ####  #      # ###### #     #  ####  #####  ###### ###### ######   #
//===========================================================================================================
    class PupilNodelet: public nodelet::Nodelet{
    private:

        /// Ros NodeHandle
        ros::NodeHandle _nh;

        // 30Hz world
        // 30Hz depth
        // 120Hz Eye
        // 120Hz Gaze
        // Total: 300Hz (NOTE: pub_time must provide higher frequency than 300Hz)
        /// Receive period
        float recv_time = 0.002; //500Hz

        /// Publishing period for the images only! Gaze is published at max frequency
        // float pub_time = 0.67; //1.5Hz
        float pub_time = 0.05; //1.5Hz


        /// Flag used by publishCallback to signal new transmission of Eye Image and Frame
        bool processEye = false;
        /// Flag used by publishCallback to signal new transmission of World Image and Frame
        bool processWorld = false;
        /// Flag used by publishCallback to signal new transmission of Depth Image and Frame
        bool processDepth = false;


        /// Timer that inforces the desired publishing frequency
        ros::Timer recv_timer;
        ros::Timer pub_timer;

        /// json object created from the [r200_info_file]
        nlohmann::json r200_info;
        /// camera configuration saved using the [save_camera] node
        const std::string r200_info_file = "/home/johnmper/.ROSData/pupil/r200_info.json";

        /// image transport object to publish the received images from pupil software
        image_transport::ImageTransport it;


        /// Message to popuulate with information extracted from PupilCapture with ZMQ
        pupil_msgs::gaze_datum gaze;
        /// Ros publisher for transmission of the gaze_datum message
        ros::Publisher gaze_pub;


        /// **Eye** Message with camera info to publish into ROS
        sensor_msgs::CameraInfo eye_info;
        /// **Eye** Message to publish in ROS
        pupil_msgs::frame eye_frame;
        /// **Eye** BGR8 EYE Image, probably not necessary to publish (and it could be gray image)
        sensor_msgs::Image eye_image;

        /// **Eye** [sensor_msgs/CameraInfo] publisher
        ros::Publisher eye_info_pub;
        /// **Eye** [pupil_msgs/frame] publisher
        ros::Publisher eye_frame_pub;
        /// **Eye** [sensor_msgs/Image] publisher
        image_transport::Publisher eye_image_pub;


        /// **World** Message with camera info to publish into ROS
        sensor_msgs::CameraInfo world_info;
        /// **World** Message to publish in ROS
        pupil_msgs::frame world_frame;
        /// **World** BGR8 Image of the world
        sensor_msgs::Image world_image;

        /// **World** World Image [sensor_msgs/CameraInfo] publisher
        ros::Publisher world_info_pub;
        /// **World** World Image [pupil_msgs/frame] publisher
        ros::Publisher world_frame_pub;
        /// **World** World Image [sensor_msgs/Image] publisher
        image_transport::Publisher world_image_pub;


        /// **Depth** Message with camera info to publish into ROS
        sensor_msgs::CameraInfo depth_info;
        /// **Depth** Message to publish in ROS
        pupil_msgs::frame depth_frame;
        /// **Depth** MONO16 Depth image
        sensor_msgs::Image depth_image;

        /// **Depth** Depth Image [sensor_msgs/CameraInfo] publisher
        ros::Publisher depth_info_pub;
        /// **Depth** Depth Image [pupil_msgs/frame] publisher
        ros::Publisher depth_frame_pub;
        /// **Depth** Depth Image [sensor_msgs/Image] publisher
        image_transport::Publisher depth_image_pub;


        /// ZMQ socket for receiving messages
        std::unique_ptr<zmq::socket_t> sub;
        /// ZMQ context
        std::unique_ptr<zmq::context_t> ctx;
        /// ZMQ socket used to contact the Pupil Remote plugin and establish initial connection
        std::unique_ptr<zmq::socket_t> remote;

        const std::vector<std::string> topic_names={"gaze.","frame."};

        /// msgpack element needed for conversion
        msgpack::object_handle oh;
        /// msgpack element needed for conversion, uses convert<> method to parse/convert the received information
        msgpack::object deserialized;

        /// TF broadcaster to create the internal R200 camera and Eye camera various links
        tf2_ros::StaticTransformBroadcaster tf_broadcaster;

        geometry_msgs::TransformStamped tf_camera_msg;
        geometry_msgs::TransformStamped tf_color_msg;
        geometry_msgs::TransformStamped tf_depth_msg;
        geometry_msgs::TransformStamped tf_optical_color_msg;
        geometry_msgs::TransformStamped tf_optical_depth_msg;

        /// Name of the **base** camera link of **R200** camera
        const std::string DEFAULT_BASE_FRAME_ID = "r200_camera_link";
        /// Name of the **world** camera link of the **R200** camera
        const std::string DEFAULT_COLOR_FRAME_ID = "r200_color_frame";
        /// Name of the **depth** camera link of the **R200** camera
        const std::string DEFAULT_DEPTH_FRAME_ID = "r200_depth_frame";
        /// Name of the **world** optical link of the **R200** camera
        const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "rgb_optical_frame";
        /// Name of the **depth** optical link of the **R200** camera
        const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "depth_optical_frame";

        /// Name of the **base** camera link of the **Eye** camera
        const std::string DEFAULT_EYE_FRAME_ID = "eye_camera_link";

        double start_time;
        std::deque<sync_header> gaze_queue;
        std::deque<sync_header> eye_queue;

//===========================================================================================================
//       #####
//      #     #  ####  #    #  ####  ##### #####  #    #  ####  #####  ####  #####
//      #       #    # ##   # #        #   #    # #    # #    #   #   #    # #    #
//      #       #    # # #  #  ####    #   #    # #    # #        #   #    # #    #
//      #       #    # #  # #      #   #   #####  #    # #        #   #    # #####
//      #     # #    # #   ## #    #   #   #   #  #    # #    #   #   #    # #   #
//       #####   ####  #    #  ####    #   #    #  ####   ####    #    ####  #    #
//===========================================================================================================
    public:
        PupilNodelet() :  it(_nh){

            // Multiple socket initializations in class initilization a.k.a:
            //      PupilNodelet() : ctx(1), sub(...), remote(...), it(_nh){ ...
            // doesnt work, dont know why, probably errors with pluginlib initialization procedure

            ctx.reset(new zmq::context_t(1));
            sub.reset(new zmq::socket_t(*ctx,ZMQ_SUB));
            remote.reset(new zmq::socket_t(*ctx,ZMQ_REQ));

            // // Connect to Pupil Remote Plugin!
            remote->connect("tcp://localhost:50020");
            //
            // // Asks Pupil Remote Plugin for the Pupil IPC_SUB_URL
            const std::string ipc_sub_command = {"SUB_PORT"};
            zmq::message_t request(ipc_sub_command.size());
            zmq::message_t response;
            //
            memcpy(request.data(), ipc_sub_command.c_str(), ipc_sub_command.size());
            remote->send(request);
            remote->recv(&response);

            // Subscribe to Pupil ICP_SUB_URL
            std::string sub_url = "tcp://localhost:" + std::string(static_cast<char*>(response.data()),response.size());
            std::cout << "Connecting to: " << sub_url << std::endl;
            sub->connect(sub_url.c_str());

            // TODO: ERROR catching!
            for(const auto& it : topic_names){
                std::cout << "Subscribing to " << it << "("<<it.size()<<")" <<std::endl;
                sub->setsockopt(ZMQ_SUBSCRIBE, it.c_str(), it.size());
            }

            gaze_pub = _nh.advertise<pupil_msgs::gaze_datum>("pupil/gaze_0/datum",1);
            eye_image_pub = it.advertise("pupil/eye_0/image_raw", 1);
            world_image_pub = it.advertise("pupil/world/image_raw", 1);
            depth_image_pub = it.advertise("pupil/depth/image_raw", 1);

            eye_frame_pub = _nh.advertise<pupil_msgs::frame>("pupil/eye_0/frame",1);
            world_frame_pub = _nh.advertise<pupil_msgs::frame>("pupil/world/frame",1);
            depth_frame_pub = _nh.advertise<pupil_msgs::frame>("pupil/depth/frame",1);

            eye_info_pub = _nh.advertise<sensor_msgs::CameraInfo>("pupil/eye_0/camera_info",1);
            world_info_pub = _nh.advertise<sensor_msgs::CameraInfo>("pupil/world/camera_info",1);
            depth_info_pub = _nh.advertise<sensor_msgs::CameraInfo>("pupil/depth/camera_info",1);

            // Open stored information about the cameras
            std::ifstream info_file(r200_info_file);
            if(info_file.fail()){
                ROS_FATAL_STREAM("Camera configuration file doesnt exist.");
            }
            info_file >> r200_info;

            // Publish the various static TF transforms
            // NOTE: necessary for the [save_camera] node used in the creation of the camera_info.json file to
            // have the save link names/IDS!!
            tf_color_msg.header.stamp = ros::Time::now();
            tf_color_msg.header.frame_id = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["parent"];
            tf_color_msg.child_frame_id = DEFAULT_COLOR_FRAME_ID;
            tf_color_msg.transform.translation.x = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["T"]["x"];
            tf_color_msg.transform.translation.y = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["T"]["y"];
            tf_color_msg.transform.translation.z = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["T"]["z"];
            tf_color_msg.transform.rotation.x = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["R"]["x"];
            tf_color_msg.transform.rotation.y = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["R"]["y"];
            tf_color_msg.transform.rotation.z = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["R"]["z"];
            tf_color_msg.transform.rotation.w = r200_info["device_0"]["tf"][DEFAULT_COLOR_FRAME_ID]["R"]["w"];
            //std::cout << " Color Frame:\n" << tf_msg << std::endl;
            tf_broadcaster.sendTransform(tf_color_msg);

            tf_optical_color_msg.header.frame_id = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["parent"];
            tf_optical_color_msg.child_frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
            tf_optical_color_msg.transform.translation.x = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["T"]["x"];
            tf_optical_color_msg.transform.translation.y = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["T"]["y"];
            tf_optical_color_msg.transform.translation.z = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["T"]["z"];
            tf_optical_color_msg.transform.rotation.x = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["R"]["x"];
            tf_optical_color_msg.transform.rotation.y = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["R"]["y"];
            tf_optical_color_msg.transform.rotation.z = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["R"]["z"];
            tf_optical_color_msg.transform.rotation.w = r200_info["device_0"]["tf"][DEFAULT_COLOR_OPTICAL_FRAME_ID]["R"]["w"];
            //std::cout << " Color Optical:\n" << tf_msg << std::endl;
            tf_broadcaster.sendTransform(tf_optical_color_msg);

            tf_depth_msg.header.frame_id = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["parent"];
            tf_depth_msg.child_frame_id = DEFAULT_DEPTH_FRAME_ID;
            tf_depth_msg.transform.translation.x = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["T"]["x"];
            tf_depth_msg.transform.translation.y = double(r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["T"]["y"]) + 0.125;
            tf_depth_msg.transform.translation.z = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["T"]["z"];
            tf_depth_msg.transform.rotation.x = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["R"]["x"];
            tf_depth_msg.transform.rotation.y = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["R"]["y"];
            tf_depth_msg.transform.rotation.z = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["R"]["z"];
            tf_depth_msg.transform.rotation.w = r200_info["device_0"]["tf"][DEFAULT_DEPTH_FRAME_ID]["R"]["w"];
            //std::cout << " Depth Frame:\n" << tf_msg << std::endl;
            tf_broadcaster.sendTransform(tf_depth_msg);

            tf_optical_depth_msg.header.frame_id = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["parent"];
            tf_optical_depth_msg.child_frame_id = DEFAULT_DEPTH_OPTICAL_FRAME_ID;
            tf_optical_depth_msg.transform.translation.x = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["T"]["x"];
            tf_optical_depth_msg.transform.translation.y = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["T"]["y"];
            tf_optical_depth_msg.transform.translation.z = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["T"]["z"];
            tf_optical_depth_msg.transform.rotation.x = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["R"]["x"];
            tf_optical_depth_msg.transform.rotation.y = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["R"]["y"];
            tf_optical_depth_msg.transform.rotation.z = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["R"]["z"];
            tf_optical_depth_msg.transform.rotation.w = r200_info["device_0"]["tf"][DEFAULT_DEPTH_OPTICAL_FRAME_ID]["R"]["w"];
            //std::cout << " Depth Optical:\n" << tf_msg << std::endl;
            tf_broadcaster.sendTransform(tf_optical_depth_msg);

            std::cout << LOG_ID("PupilNodelet") << "Created" << std::endl;

        }
        virtual ~PupilNodelet(){
            sub->close();
            remote->close();
        }

//===========================================================================================================
//      #######        ###
//      #     # #    #  #  #    # # #####
//      #     # ##   #  #  ##   # #   #
//      #     # # #  #  #  # #  # #   #
//      #     # #  # #  #  #  # # #   #
//      #     # #   ##  #  #   ## #   #
//      ####### #    # ### #    # #   #
//===========================================================================================================
    private:
        virtual void onInit(){

            // Asks Pupil Remote Plugin for the internal timestamp, synchronization
            const std::string timestamp_command = {"t"};
            zmq::message_t time_request(timestamp_command.size());
            zmq::message_t time_response;
            //
            memcpy(time_request.data(), timestamp_command.c_str(), timestamp_command.size());
            remote->send(time_request);
            remote->recv(&time_response);
            std::cout << "Pupil Timestamp: " << std::string(static_cast<char*>(time_response.data()),time_response.size()) << std::endl;
            std::cout << "Ros Timestamp: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;

            // Using a recv_timer for publishing reduces the huge and unnecessary amount of data sent into the
            // ros system and to the node/nodelets that may come to derive from this data
            recv_timer = _nh.createTimer(ros::Duration(recv_time), &pupil_comms::PupilNodelet::recvCallback,this);
            pub_timer = _nh.createTimer(ros::Duration(pub_time), &pupil_comms::PupilNodelet::publishCallback,this);

            start_time = ros::Time::now().toSec();
            std::cout << LOG_ID("PupilNodelet") << "Initialization Complete" << std::endl;

        }//end onInit

//===========================================================================================================
//      ######                        #####
//      #     # ######  ####  #    # #     #   ##   #      #      #####    ##    ####  #    #
//      #     # #      #    # #    # #        #  #  #      #      #    #  #  #  #    # #   #
//      ######  #####  #      #    # #       #    # #      #      #####  #    # #      ####
//      #   #   #      #      #    # #       ###### #      #      #    # ###### #      #  #
//      #    #  #      #    #  #  #  #     # #    # #      #      #    # #    # #    # #   #
//      #     # ######  ####    ##    #####  #    # ###### ###### #####  #    #  ####  #    #
//===========================================================================================================
        void recvCallback(const ros::TimerEvent& event){
            // NOTE:
            // Could be an while Loop if this wasnt a nodelet!
            // So just be sure recvCallback frequency is higher than the pupil send frequency
            // CHANGED:

            processEye = true;
            processWorld = true;
            processDepth = true;
            recvMsg();
        }

//===========================================================================================================
//      ######                                        #####
//      #     # #    # #####  #      #  ####  #    # #     #   ##   #      #      #####    ##    ####  #    #
//      #     # #    # #    # #      # #      #    # #        #  #  #      #      #    #  #  #  #    # #   #
//      ######  #    # #####  #      #  ####  ###### #       #    # #      #      #####  #    # #      ####
//      #       #    # #    # #      #      # #    # #       ###### #      #      #    # ###### #      #  #
//      #       #    # #    # #      # #    # #    # #     # #    # #      #      #    # #    # #    # #   #
//      #        ####  #####  ###### #  ####  #    #  #####  #    # ###### ###### #####  #    #  ####  #    #
//===========================================================================================================
        void publishCallback(const ros::TimerEvent& event){
            // Used to limit the actual ROS topic publishing frequency
            processWorld = true;
            processEye = true;
            // processDepth is toogled in recvWorld() function so they are sincronized
            //(Depth message is always received after World)
            tf_broadcaster.sendTransform(tf_color_msg);
            tf_broadcaster.sendTransform(tf_optical_color_msg);
            tf_broadcaster.sendTransform(tf_depth_msg);
            tf_broadcaster.sendTransform(tf_optical_depth_msg);
        }


//===========================================================================================================
//   ######                                            #     #
//   #     # #####   ####   ####  ######  ####   ####  ##   ##  ####   ####
//   #     # #    # #    # #    # #      #      #      # # # # #      #    #
//   ######  #    # #    # #      #####   ####   ####  #  #  #  ####  #
//   #       #####  #    # #      #           #      # #     #      # #  ###
//   #       #   #  #    # #    # #      #    # #    # #     # #    # #    #
//   #       #    #  ####   ####  ######  ####   ####  #     #  ####   ####
//===========================================================================================================
        void recvMsg(){
            static zmq::message_t msg_topic;
            // Receive the topic name (in a blocking way!)
            sub->recv(&msg_topic);
            std::string ss = std::string(static_cast<char*>(msg_topic.data()),msg_topic.size());
            if(ss.compare("gaze.3d.0.")==0){
                // std::cerr << " [gaze.3d.0.] ";
                recvGaze3D();
            }
            else if(ss.compare("frame.eye.0")==0){
                // std::cerr << " [frame.eye.0] ";
                recvFrameEye();
            }
            else if(ss.compare("frame.world")==0){
                // std::cerr << " [frame.world] ";
                recvFrameWorld();
            }
            else if(ss.compare("frame.depth")==0){
                // std::cerr << " [frame.depth] ";
                recvFrameDepth();
            }

        }

//===========================================================================================================
//      ######                                             #####                        #####  ######
//      #     # #####   ####   ####  ######  ####   ####  #     #   ##   ###### ###### #     # #     #
//      #     # #    # #    # #    # #      #      #      #        #  #      #  #            # #     #
//      ######  #    # #    # #      #####   ####   ####  #  #### #    #    #   #####   #####  #     #
//      #       #####  #    # #      #           #      # #     # ######   #    #            # #     #
//      #       #   #  #    # #    # #      #    # #    # #     # #    #  #     #      #     # #     #
//      #       #    #  ####   ####  ######  ####   ####   #####  #    # ###### ######  #####  ######
//===========================================================================================================
        void recvGaze3D(){
            // ZMQ message buffer for the gaze_datum
            static zmq::message_t msg_gaze;

            // Receive Actual Gaze Data
            sub->recv(&msg_gaze);

            // Update sequence
            ++gaze.header.seq;

            // Publish only if necessary
            if(gaze_pub.getNumSubscribers() != 0){
                // Process and populate pupil_msgs::gaze object
                oh = msgpack::unpack((const char*)msg_gaze.data(),msg_gaze.size());
                deserialized = oh.get();
                deserialized.convert<pupil_msgs::gaze_datum>(gaze);

                if(gaze_queue.size() >= eye_queue.size()){
                    gaze.header.stamp = ros::Time::now();
                    gaze_queue.push_back({gaze.header,(uint64_t)gaze.base_data[0].index});
                }
                else{
                    // Used to synchronize the eye frame with the pupil_datum indexes
                    for(unsigned int ii=eye_queue.size(); ii > 0; --ii){
                        if(eye_queue.front().index < gaze.base_data[0].index){
                            eye_queue.pop_front();
                        }
                        else{
                            gaze.header.stamp = eye_queue[0].header.stamp;
                            eye_queue.pop_front();
                            break;
                        }
                    }

                }

                // std::cout << gaze_queue.size() << " | " << eye_queue.size() << std::endl;
                if( gaze_queue.size() > 10){
                    gaze_queue.clear();
                }
                // Publish gaze data
                gaze_pub.publish(gaze);

            }

        }

//===========================================================================================================
//      ######                                            #######                             #######
//      #     # #####   ####   ####  ######  ####   ####  #       #####    ##   #    # ###### #       #   # ######
//      #     # #    # #    # #    # #      #      #      #       #    #  #  #  ##  ## #      #        # #  #
//      ######  #    # #    # #      #####   ####   ####  #####   #    # #    # # ## # #####  #####     #   #####
//      #       #####  #    # #      #           #      # #       #####  ###### #    # #      #         #   #
//      #       #   #  #    # #    # #      #    # #    # #       #   #  #    # #    # #      #         #   #
//      #       #    #  ####   ####  ######  ####   ####  #       #    # #    # #    # ###### #######   #   ######
//===========================================================================================================
        void recvFrameEye(){
            // ZMQ message buffer for the image info structure
            static zmq::message_t msg_frame_info;
            // ZMQ message buffer for the image data
            static zmq::message_t msg_frame_data;


            // Receive multipart message with Eye Camera Frame Data and actual Image
            sub->recv(&msg_frame_info);
            sub->recv(&msg_frame_data);


            if(!processEye){
                return;
            }
            // Toggle processEye, wait until publishCallback toggles it to process the received message
            processEye = false;

            // Update sequence even if no subscribers exist!
            ++eye_info.header.seq;

            // Publish only if necessary!
            if(eye_info_pub.getNumSubscribers() != 0
                    || eye_frame_pub.getNumSubscribers() != 0
                    || eye_image_pub.getNumSubscribers() != 0 ){
                // Process eye stream info object
                oh = msgpack::unpack((const char*)msg_frame_info.data(),msg_frame_info.size());
                deserialized = oh.get();
                deserialized.convert<pupil_msgs::frame>(eye_frame);

                // Populate Eye [sensor_msgs::camera_info] message
                eye_info.distortion_model = "plumb_bob"; // also called "MODIFIED_BROWN_CONRADY"
                eye_info.width = eye_frame.width;
                eye_info.height = eye_frame.height;
                // Eye camera uses a Dummy intrinsic model with the following parameters
                eye_info.K[0] = 1000;
                eye_info.K[4] = 1000;
                eye_info.K[2] = eye_frame.width/2.0;
                eye_info.K[5] = eye_frame.height/2.0;
                eye_info.D = {0,0,0,0,0};   // No distortion

                // Populate the Eye [sensor_msgs::Image] message
                eye_image.step = (uint32_t)(3*eye_frame.width);
                eye_image.encoding = sensor_msgs::image_encodings::BGR8;
                eye_image.width = eye_frame.width;
                eye_image.height = eye_frame.height;
                // Process eye stream actual image
                sensor_msgs::fillImage( eye_image,
                    eye_image.encoding,
                    eye_image.height, // height
                    eye_image.width, // width
                    eye_image.step, // stepSize
                    msg_frame_data.data());
                    eye_image.header.frame_id = DEFAULT_EYE_FRAME_ID;

                // Populate the above messages headers
                if(eye_queue.size() >= gaze_queue.size()){
                    eye_info.header.stamp = ros::Time::now();

                    eye_queue.push_back({eye_info.header,(uint64_t)eye_frame.index});
                }
                else{
                    // Used to synchronize the eye frame with the pupil_datum indexes

                    for(unsigned int ii = gaze_queue.size(); ii > 0; --ii){
                        if(gaze_queue.front().index < eye_frame.index){
                            gaze_queue.pop_front();
                        }
                        else{
                            eye_info.header.stamp = gaze_queue[0].header.stamp;
                            gaze_queue.pop_front();
                            break;
                        }
                    }
                }
                if( eye_queue.size() > 10){
                    eye_queue.clear();
                }


                eye_info.header.frame_id = DEFAULT_EYE_FRAME_ID;
                eye_image.header = eye_info.header;
                eye_info_pub.publish(eye_info);
                eye_frame_pub.publish(eye_frame);
                eye_image_pub.publish(eye_image);

                // std::cerr << eye_image.header.stamp.toSec()-start_time << '\n';
            }

        }

//===========================================================================================================
//      ######                                            #######                             #     #
//      #     # #####   ####   ####  ######  ####   ####  #       #####    ##   #    # ###### #  #  #  ####  #####  #      #####
//      #     # #    # #    # #    # #      #      #      #       #    #  #  #  ##  ## #      #  #  # #    # #    # #      #    #
//      ######  #    # #    # #      #####   ####   ####  #####   #    # #    # # ## # #####  #  #  # #    # #    # #      #    #
//      #       #####  #    # #      #           #      # #       #####  ###### #    # #      #  #  # #    # #####  #      #    #
//      #       #   #  #    # #    # #      #    # #    # #       #   #  #    # #    # #      #  #  # #    # #   #  #      #    #
//      #       #    #  ####   ####  ######  ####   ####  #       #    # #    # #    # ######  ## ##   ####  #    # ###### #####
//===========================================================================================================
        void recvFrameWorld(){
            // ZMQ message buffer for the image info structure
            static zmq::message_t msg_frame_info;
            // ZMQ message buffer for the image data
            static zmq::message_t msg_frame_data;

            // Receive multipart message with World Camera Frame Data and actual Image
            sub->recv(&msg_frame_info);
            sub->recv(&msg_frame_data);

            // Flag toggled in publishCallback()!
            if(!processWorld){
                return;
            }
            // Toggle processWorld and activate the depth receive, synchronizes World and Depth Message
            // (Depth is sent after World in Pupil Capture!)
            processWorld = false;
            processDepth = true;

            // Update message sequence even if no subscriber exists
            ++world_info.header.seq;

            // Process and Publish the World messages only if necessary
            if(world_image_pub.getNumSubscribers() != 0
                    || world_info_pub.getNumSubscribers() != 0
                    || world_frame_pub.getNumSubscribers() != 0 ){
                // Process World stream info object
                oh = msgpack::unpack((const char*)msg_frame_info.data(),msg_frame_info.size());
                deserialized = oh.get();
                deserialized.convert<pupil_msgs::frame>(world_frame);

                world_info.distortion_model = "plumb_bob"; // also called "MODIFIED_BROWN_CONRADY"
                world_info.width = world_frame.width;
                world_info.height = world_frame.height;
                world_info.K[0] = r200_info["device_0"]["color"][std::to_string(world_frame.width)+"x"+std::to_string(world_frame.height)]["fx"];
                world_info.K[4] = r200_info["device_0"]["color"][std::to_string(world_frame.width)+"x"+std::to_string(world_frame.height)]["fy"];
                world_info.K[2] = r200_info["device_0"]["color"][std::to_string(world_frame.width)+"x"+std::to_string(world_frame.height)]["ppx"];
                world_info.K[5] = 0.96*double(r200_info["device_0"]["color"][std::to_string(world_frame.width)+"x"+std::to_string(world_frame.height)]["ppy"]);
                world_info.D = r200_info["device_0"]["color"][std::to_string(world_frame.width)+"x"+std::to_string(world_frame.height)]["D"].get<std::vector<double>>();

                world_info.P[0] = world_info.K[0];
                world_info.P[2] = world_info.K[2];
                world_info.P[3] = 0;
                world_info.P[5] = world_info.K[4];
                world_info.P[6] = world_info.K[5];
                world_info.P[7] = 0;
                world_info.P[10] = 1;
                world_info.P[11] = 0;

                world_image.step = (uint32_t)(3*world_frame.width);
                world_image.encoding = sensor_msgs::image_encodings::BGR8;
                world_image.width = world_frame.width;
                world_image.height = world_frame.height;
                // Process world stream actual image
                sensor_msgs::fillImage( world_image,
                    world_image.encoding,
                    world_image.height, // height
                    world_image.width, // width
                    world_image.step, // stepSize
                    msg_frame_data.data());

                world_info.header.frame_id = DEFAULT_COLOR_OPTICAL_FRAME_ID;
                world_info.header.stamp = ros::Time::now();
                world_info_pub.publish(world_info);
                world_frame_pub.publish(world_frame);
                world_image.header = world_info.header;
                world_image_pub.publish(world_image);

                //std::cout << world_frame.index << "     " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() << std::endl;


                // std::cerr << world_image.header.stamp.toSec()-start_time << '\n';
            }
        }


//===========================================================================================================
//      ######                                            #######                             ######
//      #     # #####   ####   ####  ######  ####   ####  #       #####    ##   #    # ###### #     # ###### #####  ##### #    #
//      #     # #    # #    # #    # #      #      #      #       #    #  #  #  ##  ## #      #     # #      #    #   #   #    #
//      ######  #    # #    # #      #####   ####   ####  #####   #    # #    # # ## # #####  #     # #####  #    #   #   ######
//      #       #####  #    # #      #           #      # #       #####  ###### #    # #      #     # #      #####    #   #    #
//      #       #   #  #    # #    # #      #    # #    # #       #   #  #    # #    # #      #     # #      #        #   #    #
//      #       #    #  ####   ####  ######  ####   ####  #       #    # #    # #    # ###### ######  ###### #        #   #    #
//===========================================================================================================
        void recvFrameDepth(){
            // ZMQ message buffer for the image info structure
            static zmq::message_t msg_frame_info;
            // ZMQ message buffer for the image data
            static zmq::message_t msg_frame_data;

            // Receive multipart message with Depth Camera Frame Data and actual Depth Image
            sub->recv(&msg_frame_info);
            sub->recv(&msg_frame_data);

            if(!processDepth){
                return;
            }
            processDepth = false;

            // Update header sequence
            ++depth_info.header.seq;

            // Publish only if necessary!
            if(depth_image_pub.getNumSubscribers() != 0
                    || depth_info_pub.getNumSubscribers() != 0
                    || depth_frame_pub.getNumSubscribers() != 0 ){
                // Process eye stream info object
                oh = msgpack::unpack((const char*)msg_frame_info.data(),msg_frame_info.size());
                deserialized = oh.get();
                deserialized.convert<pupil_msgs::frame>(depth_frame);

                depth_info.distortion_model = "plumb_bob"; // also called "MODIFIED_BROWN_CONRADY"
                depth_info.width = depth_frame.width;
                depth_info.height = depth_frame.height;
                depth_info.K[0] = r200_info["device_0"]["depth"][std::to_string(depth_frame.width)+"x"+std::to_string(depth_frame.height)]["fx"];
                depth_info.K[4] = r200_info["device_0"]["depth"][std::to_string(depth_frame.width)+"x"+std::to_string(depth_frame.height)]["fy"];
                depth_info.K[2] = r200_info["device_0"]["depth"][std::to_string(depth_frame.width)+"x"+std::to_string(depth_frame.height)]["ppx"];
                depth_info.K[5] = r200_info["device_0"]["depth"][std::to_string(depth_frame.width)+"x"+std::to_string(depth_frame.height)]["ppy"];
                depth_info.D    = r200_info["device_0"]["depth"][std::to_string(depth_frame.width)+"x"+std::to_string(depth_frame.height)]["D"].get<std::vector<double>>();

                depth_info.P[0]  = depth_info.K[0];
                depth_info.P[2]  = depth_info.K[2];
                depth_info.P[3]  = 0;
                depth_info.P[5]  = depth_info.K[4];
                depth_info.P[6]  = depth_info.K[5];
                depth_info.P[7]  = 0;
                depth_info.P[10] = 1;
                depth_info.P[11] = 0;

                depth_image.step = (uint32_t)(2*depth_frame.width);
                depth_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
                depth_image.width = depth_frame.width;
                depth_image.height = depth_frame.height;
                // Process world stream actual image    (NOTE: fillImage() only fills data variable!)
                sensor_msgs::fillImage( depth_image,
                    depth_image.encoding,
                    depth_image.height, // height
                    depth_image.width, // width
                    depth_image.step, // stepSize
                    msg_frame_data.data());
                    // Publish depth
                depth_info.header.frame_id = DEFAULT_DEPTH_OPTICAL_FRAME_ID;
                // World is received before depth so use its ros time
                depth_info.header.stamp = world_info.header.stamp;
                depth_image.header = depth_info.header;
                depth_info_pub.publish(depth_info);
                depth_frame_pub.publish(depth_frame);
                depth_image_pub.publish(depth_image);

                // std::cerr << depth_image.header.stamp.toSec()-start_time << '\n';
            }
        }

    };

    PLUGINLIB_DECLARE_CLASS(pupil_comms, PupilNodelet, pupil_comms::PupilNodelet, nodelet::Nodelet);

}
