// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <omp.h>
#include <iostream>
#include <sstream>
#include <functional>
#include <iomanip>
#include <map>
#include <atomic>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


namespace pupil_comms
{

    const rs::option start_options[] = {
        rs::option::r200_disparity_multiplier,
        rs::option::r200_disparity_shift
    };
    const std::string start_options_names[] = {
        "r200_disparity_multiplier",
        "r200_disparity_shift"
    };

    // List of options that can be queried at run time
    const rs::option runtime_options[] = {
        rs::option::color_backlight_compensation,
        rs::option::color_brightness,
        rs::option::color_contrast,
        rs::option::color_exposure,
        rs::option::color_gain,
        rs::option::color_gamma,
        rs::option::color_hue,
        rs::option::color_saturation,
        rs::option::color_sharpness,
        rs::option::color_white_balance,
        rs::option::color_enable_auto_exposure,
        rs::option::color_enable_auto_white_balance,
        rs::option::r200_lr_auto_exposure_enabled,
        rs::option::r200_lr_gain,
        rs::option::r200_lr_exposure,
        rs::option::r200_emitter_enabled,
        rs::option::r200_depth_units,
        rs::option::r200_depth_clamp_min,
        rs::option::r200_depth_clamp_max,
        rs::option::r200_auto_exposure_mean_intensity_set_point,    // Requires r200_lr_auto_exposure_enabled set to 1
        rs::option::r200_auto_exposure_bright_ratio_set_point,      // Requires r200_lr_auto_exposure_enabled set to 1
        rs::option::r200_auto_exposure_kp_gain,                     // Requires r200_lr_auto_exposure_enabled set to 1
        rs::option::r200_auto_exposure_kp_exposure,
        rs::option::r200_auto_exposure_kp_dark_threshold,
        rs::option::r200_auto_exposure_top_edge,
        rs::option::r200_auto_exposure_bottom_edge,
        rs::option::r200_auto_exposure_left_edge,
        rs::option::r200_auto_exposure_right_edge,
        rs::option::r200_depth_control_estimate_median_decrement,   // Value to subtract when estimating the median of the correlation of the surface
        rs::option::r200_depth_control_estimate_median_increment,   // Value to add when estimating the median of the correlation of the surface
        rs::option::r200_depth_control_median_threshold,
        rs::option::r200_depth_control_score_minimum_threshold,
        rs::option::r200_depth_control_score_maximum_threshold,
        rs::option::r200_depth_control_texture_count_threshold,
        rs::option::r200_depth_control_texture_difference_threshold,
        rs::option::r200_depth_control_second_peak_threshold,
        rs::option::r200_depth_control_neighbor_threshold,
        rs::option::r200_depth_control_lr_threshold
    };
    // List of options that can be queried at run time
    const std::string runtime_options_names[] = {
        "color_backlight_compensation",
        "color_brightness",
        "color_contrast",
        "color_exposure",
        "color_gain",
        "color_gamma",
        "color_hue",
        "color_saturation",
        "color_sharpness",
        "color_white_balance",
        "color_enable_auto_exposure",
        "color_enable_auto_white_balance",
        "r200_lr_auto_exposure_enabled",
        "r200_lr_gain",
        "r200_lr_exposure",
        "r200_emitter_enabled",
        "r200_depth_units",
        "r200_depth_clamp_min",
        "r200_depth_clamp_max",
        "r200_auto_exposure_mean_intensity_set_point",    // Requires r200_lr_auto_exposure_enabled set to 1
        "r200_auto_exposure_bright_ratio_set_point",      // Requires r200_lr_auto_exposure_enabled set to 1
        "r200_auto_exposure_kp_gain",                     // Requires r200_lr_auto_exposure_enabled set to 1
        "r200_auto_exposure_kp_exposure",
        "r200_auto_exposure_kp_dark_threshold",
        "r200_auto_exposure_top_edge",
        "r200_auto_exposure_bottom_edge",
        "r200_auto_exposure_left_edge",
        "r200_auto_exposure_right_edge",
        "r200_depth_control_estimate_median_decrement",   // Value to subtract when estimating the median of the correlation of the surface
        "r200_depth_control_estimate_median_increment",   // Value to add when estimating the median of the correlation of the surface
        "r200_depth_control_median_threshold",
        "r200_depth_control_score_minimum_threshold",
        "r200_depth_control_score_maximum_threshold",
        "r200_depth_control_texture_count_threshold",
        "r200_depth_control_texture_difference_threshold",
        "r200_depth_control_second_peak_threshold",
        "r200_depth_control_neighbor_threshold",
        "r200_depth_control_lr_threshold"
    };

    const std::string DEFAULT_SERIAL_NO = "";

    const int DEPTH_WIDTH = 480;
    const int DEPTH_HEIGHT = 360;
    const int COLOR_WIDTH = 1920;
    const int COLOR_HEIGHT = 1080;
    const int DEPTH_FPS = 30;
    const int COLOR_FPS = 30;

    const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const float MILLIMETER_METERS  = 0.001;
    const float MAX_Z = 2.5f; // meters
    const float MIN_Z = 0.3f;

    const std::string DEFAULT_BASE_FRAME_ID = "r200_camera_link";
    const std::string DEFAULT_DEPTH_FRAME_ID = "r200_depth_frame";
    const std::string DEFAULT_COLOR_FRAME_ID = "r200_color_frame";
    const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "depth_optical_frame";
    const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "rgb_optical_frame";

    class R200Nodelet: public nodelet::Nodelet
    {
    public:
        R200Nodelet() : camera_started(false), initialized_time(false){
            // Types for depth stream
            format[rs::stream::depth] = rs::format::z16;   // libRS type
            image_format[rs::stream::depth] = CV_16UC1;    // CVBridge type
            encoding[rs::stream::depth] = sensor_msgs::image_encodings::MONO16; // ROS message type
            unit_step_size[rs::stream::depth] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
            stream_name[rs::stream::depth] = "depth";

            // Types for color stream
            format[rs::stream::color] = rs::format::rgb8;  // libRS type
            image_format[rs::stream::color] = CV_8UC3;     // CVBridge type
            encoding[rs::stream::color] = sensor_msgs::image_encodings::RGB8; // ROS message type
            unit_step_size[rs::stream::color] = sizeof(unsigned char) * 3; // sensor_msgs::ImagePtr row step size
            stream_name[rs::stream::color] = "color";

            // Initialization of the COLOR image storage (all 0)
            image[rs::stream::color] = cv::Mat(COLOR_HEIGHT, COLOR_WIDTH, image_format[rs::stream::color], cv::Scalar(0, 0, 0));
            // Initialization of the DEPTH image storage (all 0)
            image[rs::stream::depth] = cv::Mat(DEPTH_HEIGHT, DEPTH_WIDTH, image_format[rs::stream::depth], cv::Scalar(0, 0, 0));

        }

        virtual ~R200Nodelet(){
            if (camera_started) {
                device->stop();
            }

            usleep(250);
            ctx.reset();
        }

    private:
        virtual void onInit(){
            getParameters();

            if (!setupDevice())
                return;

            setupPublishers();
            setupStreams();
            publishStaticTransforms();
        }//end onInit


        void getParameters(){
            pnh_ = getPrivateNodeHandle();

            pnh_.param("serial_no", serial_n, DEFAULT_SERIAL_NO);

            pnh_.param("depth_width", width[rs::stream::depth], DEPTH_WIDTH);
            pnh_.param("depth_height", height[rs::stream::depth], DEPTH_HEIGHT);
            pnh_.param("depth_fps", fps[rs::stream::depth], DEPTH_FPS);

            pnh_.param("color_width", width[rs::stream::color], COLOR_WIDTH);
            pnh_.param("color_height", height[rs::stream::color], COLOR_HEIGHT);
            pnh_.param("color_fps", fps[rs::stream::color], COLOR_FPS);

            pnh_.param("base_frame_id", base_frame_id, DEFAULT_BASE_FRAME_ID);
            pnh_.param("depth_frame_id", frame_id[rs::stream::depth], DEFAULT_DEPTH_FRAME_ID);
            pnh_.param("color_frame_id", frame_id[rs::stream::color], DEFAULT_COLOR_FRAME_ID);
            pnh_.param("depth_optical_frame_id", optical_frame_id[rs::stream::depth], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
            pnh_.param("color_optical_frame_id", optical_frame_id[rs::stream::color], DEFAULT_COLOR_OPTICAL_FRAME_ID);

        }//end getParameters


        bool setupDevice(){
            // Find connected realsense devices
            // Note: Not prep for multiple devices
            ctx.reset(new rs::context());
            int num_rs = ctx->get_device_count();
            if (num_rs == 0){
                ROS_ERROR_STREAM("error : no RealSense devices found.");
                ctx.reset();
                return false;
            }
            else if(num_rs > 1){
                ROS_ERROR_STREAM("error : multiple Realsense devices found.");
                ctx.reset();
                return false;
            }

            // Store realsense device
            device = ctx->get_device(0);
            serial_n == std::string(device->get_serial());

            auto device_name = device->get_name();
            ROS_INFO_STREAM(device_name << ", serial_no: " << serial_n);
            if ((std::string(device_name).find("R200") == std::string::npos)){
                ROS_ERROR_STREAM("error: This ROS node supports R200 only.");
                ctx.reset();
                return false;
            }

            ROS_INFO_STREAM("Finished setup of device.");
            return true;
        }


        void setupPublishers(){
            // Image transport handle
            image_transport::ImageTransport image_transport(getNodeHandle());

            // COLOR Stream Topics
            image_pub[rs::stream::color] = image_transport.advertise("pupil/world/image_raw", 1);
            info_pub[rs::stream::color] = node.advertise< sensor_msgs::CameraInfo >("pupil/world/camera_info", 1);

            // DEPTH Stream Topics
            image_pub[rs::stream::depth] = image_transport.advertise("pupil/depth/image_raw", 1);
            info_pub[rs::stream::depth] = node.advertise< sensor_msgs::CameraInfo >("pupil/depth/camera_info", 1);

            // POINTCLOUD Topics
            pointcloud_pub = node.advertise<sensor_msgs::PointCloud2>("/pupil/pointcloud", 1);

            ROS_INFO_STREAM("Finished setup of publishers.");
        }


        void setupOptions(){


            double start_values[2];
            double runtime_values[38];

            // START BY QUERYING ALL THE AVAILABLE OPTIONS!!
            device->get_options(start_options,2,start_values);
            device->get_options(runtime_options,38,runtime_values);

            // PRINT THEM
            std::stringstream ss;
            ss << "R200 Internal configuration:" << std::endl;
            ss << "Start Values:" << std::endl;
            for(int ii=0; ii < 2 ; ++ii){
                ss << std::setw(8) << std::right << start_values[ii] << " : " << start_options_names[ii] <<std::endl;
            }

            ss << "Runtime Values:" << std::endl;
            for(int ii=0; ii < 38 ; ++ii){
                ss << std::setw(8) << std::right << runtime_values[ii] << " : " << runtime_options_names[ii] <<std::endl;
            }
            ROS_INFO_STREAM(ss.str());


            // DEFAULT USER SETTINGS
            // COLOR SETTINGS
            device->set_option(rs::option::color_enable_auto_exposure, 1);
            device->set_option(rs::option::color_enable_auto_white_balance, 1);
            // R200 SETTINGS
            /*
            device->set_option(rs::option::r200_lr_auto_exposure_enabled, 0);
            device->set_option(rs::option::r200_lr_gain, 400);
            device->set_option(rs::option::r200_lr_exposure, 164);
            device->set_option(rs::option::r200_emitter_enabled, 1);
            device->set_option(rs::option::r200_depth_units, 1000);
            device->set_option(rs::option::r200_depth_clamp_min, 0);
            device->set_option(rs::option::r200_depth_clamp_max, 65535);
            device->set_option(rs::option::r200_auto_exposure_mean_intensity_set_point, 512);
            device->set_option(rs::option::r200_auto_exposure_bright_ratio_set_point, 0);
            device->set_option(rs::option::r200_auto_exposure_kp_gain, 0);
            device->set_option(rs::option::r200_auto_exposure_kp_exposure, 0);
            device->set_option(rs::option::r200_auto_exposure_kp_dark_threshold, 10);
            device->set_option(rs::option::r200_auto_exposure_top_edge, 0);
            device->set_option(rs::option::r200_auto_exposure_bottom_edge, 479);
            device->set_option(rs::option::r200_auto_exposure_left_edge, 0);
            device->set_option(rs::option::r200_auto_exposure_right_edge, 639);
            device->set_option(rs::option::r200_depth_control_estimate_median_decrement, 5);
            device->set_option(rs::option::r200_depth_control_estimate_median_increment, 5);
            device->set_option(rs::option::r200_depth_control_median_threshold, 192);
            device->set_option(rs::option::r200_depth_control_score_minimum_threshold, 1);
            device->set_option(rs::option::r200_depth_control_score_maximum_threshold, 512);
            device->set_option(rs::option::r200_depth_control_texture_count_threshold, 6);
            device->set_option(rs::option::r200_depth_control_texture_difference_threshold, 24);
            device->set_option(rs::option::r200_depth_control_second_peak_threshold, 27);
            device->set_option(rs::option::r200_depth_control_neighbor_threshold, 7);
            device->set_option(rs::option::r200_depth_control_lr_threshold, 24);
            */

        }

        void setupStreams(){
            // Show and ask user if the current options are ok
            setupOptions();

            // LAMBDAS BLACK MAGIC!! MUAHAHAHHA
            stream_callback[rs::stream::color] = [this](rs::frame frame){colorFrameCallback(frame);};
            stream_callback[rs::stream::depth] = [this](rs::frame frame){depthFrameCallback(frame);};


            // Enable the COLOR stream
            device->enable_stream(rs::stream::color,
                                width[rs::stream::color],
                                height[rs::stream::color],
                                format[rs::stream::color],
                                fps[rs::stream::color]);

            // Publish info about the COLOR stream
            // NOTE: TO TEST
            getStreamCalibData(rs::stream::color);
            // Set Callback for COLOR stream
            device->set_frame_callback(rs::stream::color, stream_callback[rs::stream::color]);
            ROS_INFO_STREAM("  Enabled " << stream_name[rs::stream::color] << " stream:"
                        << " Width: " << camera_info[rs::stream::color].width
                        << " Height: " << camera_info[rs::stream::color].height
                        << " FPS: " << fps[rs::stream::color]);

            // Enable the DEPTH Stream
            device->enable_stream(rs::stream::depth,
                                width[rs::stream::depth],
                                height[rs::stream::depth],
                                format[rs::stream::depth],
                                fps[rs::stream::depth]);

            // Publish info about the DEPTH stream
            // NOTE: TO TEST
            getStreamCalibData(rs::stream::depth);
            // Set Callback for COLOR stream
            device->set_frame_callback(rs::stream::depth, stream_callback[rs::stream::depth]);
            ROS_INFO_STREAM("  Enabled " << stream_name[rs::stream::depth] << " stream:"
                        << " Width: " << camera_info[rs::stream::depth].width
                        << " Height: " << camera_info[rs::stream::depth].height
                        << " FPS: " << fps[rs::stream::depth]);

            // All streams have been enabled so start the device
            device->start();
            camera_started = true;

        }//end setupStreams

        void colorFrameCallback(const rs::frame& frame){

            // Get pointer to current color frame data
            image[rs::stream::color].data = (unsigned char*)frame.get_data();

            // Calculate time
            ros::Time tm(updateTime(frame));

            // Increment the frame counter
            ++seq[rs::stream::color];

            // Check if someone is reading the image topic, publish to it if that's the case
            if(image_pub[rs::stream::color].getNumSubscribers() != 0 ||
                 info_pub[rs::stream::color].getNumSubscribers() != 0 ){


                // Prep image message for publishing
                sensor_msgs::ImagePtr img_ptr;
                img_ptr = cv_bridge::CvImage(std_msgs::Header(), encoding[rs::stream::color], image[rs::stream::color]).toImageMsg();
                img_ptr->width = image[rs::stream::color].cols;
                img_ptr->height = image[rs::stream::color].rows;
                img_ptr->is_bigendian = false;
                img_ptr->step = image[rs::stream::color].cols * unit_step_size[rs::stream::color];
                img_ptr->header.frame_id = optical_frame_id[rs::stream::color];
                img_ptr->header.stamp = tm;
                img_ptr->header.seq = seq[rs::stream::color];

                image_pub[rs::stream::color].publish(img_ptr);

                camera_info[rs::stream::color].header.stamp = tm;
                camera_info[rs::stream::color].header.seq = seq[rs::stream::color];
                info_pub[rs::stream::color].publish(camera_info[rs::stream::color]);
            }

        }

        void depthFrameCallback(const rs::frame& frame){

            // Get pointer to current color frame data
            image[rs::stream::depth].data = (unsigned char*)frame.get_data();

            // Median Smoothing of depth frame (in place operation supported)
            //cv::medianBlur(image[rs::stream::depth],image[rs::stream::depth],5);

            // Calculate time
            ros::Time tm(updateTime(frame));

            // Increment the frame counter
            ++seq[rs::stream::depth];

            // Check if someone is reading the pointcloud topic, publish to it if that's the case
            if(pointcloud_pub.getNumSubscribers() > 0){
                publishPCTopic(tm);
            }

            // Check if someone is reading the depth image topic, publish to it if that's the case
            if(image_pub[rs::stream::depth].getNumSubscribers() > 0 ||
                 info_pub[rs::stream::depth].getNumSubscribers() > 0 ){

                // Prep image message for publishing
                sensor_msgs::ImagePtr img_ptr;
                img_ptr = cv_bridge::CvImage(std_msgs::Header(), encoding[rs::stream::depth], image[rs::stream::depth]).toImageMsg();
                img_ptr->width = image[rs::stream::depth].cols;
                img_ptr->height = image[rs::stream::depth].rows;
                img_ptr->is_bigendian = false;
                img_ptr->step = image[rs::stream::depth].cols * unit_step_size[rs::stream::depth];
                img_ptr->header.frame_id = optical_frame_id[rs::stream::depth];
                img_ptr->header.stamp = tm;
                img_ptr->header.seq = seq[rs::stream::depth];

                image_pub[rs::stream::depth].publish(img_ptr);

                camera_info[rs::stream::depth].header.stamp = tm;
                camera_info[rs::stream::depth].header.seq = seq[rs::stream::depth];
                info_pub[rs::stream::depth].publish(camera_info[rs::stream::depth]);
            }

        }

        ros::Time updateTime(const rs::frame& frame){
            if( ! initialized_time ){
                // Time hasnt been initialized..! Time to do it
                initialized_time = true;
                ros_time_base = ros::Time::now();
                camera_time_base = frame.get_timestamp();      // Returns time in miliseconds
            }
            double elapsed_camera = (frame.get_timestamp()-camera_time_base)/1000.0;
            return ros::Time(ros_time_base.toSec() + elapsed_camera);
        }


        void getStreamCalibData(rs::stream stream)
        {
            rs::intrinsics intrinsic = device->get_stream_intrinsics(stream);

            camera_info[stream].header.frame_id = optical_frame_id[stream];
            camera_info[stream].width = intrinsic.width;
            camera_info[stream].height = intrinsic.height;

            camera_info[stream].K.at(0) = intrinsic.fx;
            camera_info[stream].K.at(2) = intrinsic.ppx;
            camera_info[stream].K.at(4) = intrinsic.fy;
            camera_info[stream].K.at(5) = intrinsic.ppy;
            camera_info[stream].K.at(8) = 1;

            camera_info[stream].P.at(0) = camera_info[stream].K.at(0);
            camera_info[stream].P.at(1) = 0;
            camera_info[stream].P.at(2) = camera_info[stream].K.at(2);
            camera_info[stream].P.at(3) = 0;
            camera_info[stream].P.at(4) = 0;
            camera_info[stream].P.at(5) = camera_info[stream].K.at(4);
            camera_info[stream].P.at(6) = camera_info[stream].K.at(5);
            camera_info[stream].P.at(7) = 0;
            camera_info[stream].P.at(8) = 0;
            camera_info[stream].P.at(9) = 0;
            camera_info[stream].P.at(10) = 1;
            camera_info[stream].P.at(11) = 0;

            if (stream == rs::stream::depth)
            {
                // set depth to color translation values in Projection matrix (P)
                rs::extrinsics extrinsic = device->get_extrinsics(rs::stream::depth, rs::stream::color);
                camera_info[stream].P.at(3) = extrinsic.translation[0];     // Tx
                camera_info[stream].P.at(7) = extrinsic.translation[1];     // Ty
                camera_info[stream].P.at(11) = extrinsic.translation[2];    // Tz

                for (int i = 0; i < 9; i++)
                camera_info[stream].R.at(i) = extrinsic.rotation[i];
            }

            switch ((int32_t)intrinsic.model())
            {
                case 0:
                camera_info[stream].distortion_model = "plumb_bob";
                break;
                case 1:
                // This is the same as "modified_brown_conrady", but used by ROS
                camera_info[stream].distortion_model = "plumb_bob";
                break;
                case 2:
                camera_info[stream].distortion_model = "inverse_brown_conrady";
                break;
                case 3:
                camera_info[stream].distortion_model = "distortion_ftheta";
                break;
                default:
                camera_info[stream].distortion_model = "others";
                break;
            }

            // set R (rotation matrix) values to identity matrix
            if (stream != rs::stream::depth)
            {
                camera_info[stream].R.at(0) = 1.0;
                camera_info[stream].R.at(1) = 0.0;
                camera_info[stream].R.at(2) = 0.0;
                camera_info[stream].R.at(3) = 0.0;
                camera_info[stream].R.at(4) = 1.0;
                camera_info[stream].R.at(5) = 0.0;
                camera_info[stream].R.at(6) = 0.0;
                camera_info[stream].R.at(7) = 0.0;
                camera_info[stream].R.at(8) = 1.0;
            }

            for (int i = 0; i < 5; i++)
            {
                camera_info[stream].D.push_back(intrinsic.coeffs[i]);
            }
        }//end getStreamCalibData


        void publishStaticTransforms()
        {
            // Publish transforms for the cameras
            tf::Quaternion q_c2co;
            tf::Quaternion q_d2do;
            tf::Quaternion q_i2io;
            geometry_msgs::TransformStamped b2d_msg;
            geometry_msgs::TransformStamped d2do_msg;
            geometry_msgs::TransformStamped b2c_msg;
            geometry_msgs::TransformStamped c2co_msg;
            geometry_msgs::TransformStamped b2i_msg;
            geometry_msgs::TransformStamped i2io_msg;

            // Get the current timestamp for all static transforms
            ros::Time transform_ts_ = ros::Time::now();

            // The color frame is used as the base frame.
            // Hence no additional transformation is done from base frame to color frame.
            b2c_msg.header.stamp = transform_ts_;
            b2c_msg.header.frame_id = base_frame_id;
            b2c_msg.child_frame_id = frame_id[rs::stream::color];
            b2c_msg.transform.translation.x = 0;
            b2c_msg.transform.translation.y = 0;
            b2c_msg.transform.translation.z = 0;
            b2c_msg.transform.rotation.x = 0;
            b2c_msg.transform.rotation.y = 0;
            b2c_msg.transform.rotation.z = 0;
            b2c_msg.transform.rotation.w = 1;
            static_tf_broadcaster.sendTransform(b2c_msg);

            // Transform color frame to color optical frame
            q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
            c2co_msg.header.stamp = transform_ts_;
            c2co_msg.header.frame_id = frame_id[rs::stream::color];
            c2co_msg.child_frame_id = optical_frame_id[rs::stream::color];
            c2co_msg.transform.translation.x = 0;
            c2co_msg.transform.translation.y = 0;
            c2co_msg.transform.translation.z = 0;
            c2co_msg.transform.rotation.x = q_c2co.getX();
            c2co_msg.transform.rotation.y = q_c2co.getY();
            c2co_msg.transform.rotation.z = q_c2co.getZ();
            c2co_msg.transform.rotation.w = q_c2co.getW();
            static_tf_broadcaster.sendTransform(c2co_msg);

            // Transform base frame to depth frame
            rs::extrinsics color2depth_extrinsic = device->get_extrinsics(rs::stream::color, rs::stream::depth);
            b2d_msg.header.stamp = transform_ts_;
            b2d_msg.header.frame_id = base_frame_id;
            b2d_msg.child_frame_id = frame_id[rs::stream::depth];
            b2d_msg.transform.translation.x =  color2depth_extrinsic.translation[2];
            b2d_msg.transform.translation.y = -color2depth_extrinsic.translation[0];
            b2d_msg.transform.translation.z = -color2depth_extrinsic.translation[1];
            b2d_msg.transform.rotation.x = 0;
            b2d_msg.transform.rotation.y = 0;
            b2d_msg.transform.rotation.z = 0;
            b2d_msg.transform.rotation.w = 1;
            static_tf_broadcaster.sendTransform(b2d_msg);

            // Transform depth frame to depth optical frame
            q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
            d2do_msg.header.stamp = transform_ts_;
            d2do_msg.header.frame_id = frame_id[rs::stream::depth];
            d2do_msg.child_frame_id = optical_frame_id[rs::stream::depth];
            d2do_msg.transform.translation.x = 0;
            d2do_msg.transform.translation.y = 0;
            d2do_msg.transform.translation.z = 0;
            d2do_msg.transform.rotation.x = q_d2do.getX();
            d2do_msg.transform.rotation.y = q_d2do.getY();
            d2do_msg.transform.rotation.z = q_d2do.getZ();
            d2do_msg.transform.rotation.w = q_d2do.getW();
            static_tf_broadcaster.sendTransform(d2do_msg);
        }

        void publishPCTopic(ros::Time t)
        {
            rs::intrinsics depth_intrinsic = device->get_stream_intrinsics(rs::stream::depth);
            float depth_scale_meters = device->get_depth_scale();

            sensor_msgs::PointCloud2 msg_pointcloud;
            msg_pointcloud.header.stamp = t;
            msg_pointcloud.header.frame_id = optical_frame_id[rs::stream::depth];
            msg_pointcloud.width = depth_intrinsic.width;
            msg_pointcloud.height = depth_intrinsic.height;
            msg_pointcloud.is_dense = true;

            sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
            modifier.setPointCloud2Fields(3,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32);
            modifier.setPointCloud2FieldsByString(1, "xyz");

            for (int v = 0; v < depth_intrinsic.height; v++)
                for (int u = 0; u < depth_intrinsic.width; u++)
                {
                    float depth_point[3], scaled_depth;
                    uint16_t depth_value;
                    int depth_offset, cloud_offset;

                    // Offset into point cloud data, for point at u, v
                    cloud_offset = (v * msg_pointcloud.row_step) + (u * msg_pointcloud.point_step);

                    // Retrieve depth value, and scale it in terms of meters
                    depth_offset = (u * sizeof(uint16_t)) + (v * sizeof(uint16_t) * depth_intrinsic.width);
                    memcpy(&depth_value, &image[rs::stream::depth].data[depth_offset], sizeof(uint16_t));
                    scaled_depth = static_cast<float>(depth_value) * depth_scale_meters;
                    if (scaled_depth <= MIN_Z || scaled_depth > MAX_Z){
                        // Depth value is invalid, so zero it out.
                        depth_point[0] = 0.0f;
                        depth_point[1] = 0.0f;
                        depth_point[2] = 0.0f;
                    }
                    else{
                        // Convert depth image to points in 3D space
                        float depth_pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
                        rs_deproject_pixel_to_point(depth_point, &depth_intrinsic, depth_pixel, scaled_depth);
                    }

                    // Assign 3d point
                    memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[0].offset], &depth_point[0], sizeof(float)); // X
                    memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[1].offset], &depth_point[1], sizeof(float)); // Y
                    memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[2].offset], &depth_point[2], sizeof(float)); // Z
                } // for

                pointcloud_pub.publish(msg_pointcloud);
            }


        private:
            ros::NodeHandle node, pnh_;
            bool camera_started;
            std::unique_ptr< rs::context > ctx;
            rs::device *device;

            std::string serial_n;


            std::map<rs::stream, int> width;
            std::map<rs::stream, int> height;
            std::map<rs::stream, int> fps;
            std::map<rs::stream, std::string> stream_name;
            tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;

            std::map<rs::stream, image_transport::Publisher> image_pub;
            std::map<rs::stream, int> image_format;
            std::map<rs::stream, rs::format> format;
            std::map<rs::stream, ros::Publisher> info_pub;
            std::map<rs::stream, cv::Mat> image;
            std::map<rs::stream, std::string> encoding;
            std::string base_frame_id;
            std::map<rs::stream, std::string> frame_id;
            std::map<rs::stream, std::string> optical_frame_id;
            std::map<rs::stream, int> seq;
            std::map<rs::stream, int> unit_step_size;
            std::map<rs::stream, std::function<void(rs::frame)>> stream_callback;
            std::map<rs::stream, sensor_msgs::CameraInfo> camera_info;
            ros::Publisher pointcloud_pub;
            bool initialized_time;
            double camera_time_base;
            ros::Time ros_time_base;

        };//end class

        PLUGINLIB_DECLARE_CLASS(pupil_comms, R200Nodelet, pupil_comms::R200Nodelet, nodelet::Nodelet);

    }//end namespace
