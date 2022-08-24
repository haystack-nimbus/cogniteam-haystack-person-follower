

#ifndef COGNITEAM_PERSON_FOLLOWER_H
#define COGNITEAM_PERSON_FOLLOWER_H

#include <opencv2/core/utility.hpp>
//#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/tracking.hpp>
//#include <opencv2/face.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <queue>
#include "ros_compat.h"
#include <image_transport/image_transport.h>

// yakir
#include <depth_image_proc/depth_conversions.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

#include <string>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <chrono>
#include <limits>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include "../opencv/cv-helpers.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <leg_tracker/LegArray.h>

#include "../include/detectnet.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

#define PERSON_CLASS_ID (1)
#define PERSON_THRESHOLD (0.5)

#define UNKNOWN (-1)
#define FREE (0)
#define BLOCKED (100)

enum FollowerState
{
    IDLE,
    FOLLOW,
    FIND_LOST_CAMERA_TARGET_BY_LIDAR,
    PREDICT_LOST_TARGET,
    STOP
};

struct Person
{

    cv::Rect box_;

    cv::Point2d centerMassPix_;

    float distanceM_ = -1.0;

    geometry_msgs::PointStamped location_;

    float degAngle_ = 0.0;
};

class CogniteamPersonFollower
{

public:
    CogniteamPersonFollower()
    {       

        ros::NodeHandle nodePrivate("~");

        nodePrivate.setParam("/person_follower/status", string("INITIALIZING"));
        status_= "INITIALIZING";
        
        nodePrivate.setParam("/person_follower/person_follower_set_enable", false);
        setEnable_ = false;
        //setEnable_ = true;


        nodePrivate.param("/detection_img/compressed/jpeg_quality", 20);

        nodePrivate.param("follow_without_drive", follow_without_drive_, false);

        nodePrivate.param("robot_radius", robotRadius_, (float)0.3);

        nodePrivate.param("target_offset", targetOffset_, (float)0.5);

        nodePrivate.param("angle_deg_target_no_rotation", angleNotRotate_, (float)15.0);

        nodePrivate.param("max_distance_init_global_target", maxDistInitGlobalTarget_, (float)2.0);

        nodePrivate.param("max_distance_rotate_in_place", maxDistanceRotateInPlace_, (float)1.5);

        nodePrivate.param("max_distance_to_follow", max_distance_to_follow_, (float)5.0);

        nodePrivate.param("max_linear_speed", max_linear_speed_, (float)0.8);

        nodePrivate.param("min_linear_speed", min_linear_speed_, (float)0.2);

        nodePrivate.param("max_angular_speed", max_angular_speed_, (float)1.2);

        nodePrivate.param("min_angular_speed", min_angular_speed_, (float)0.5);

        nodePrivate.param("max_dist_current_global_and_prev_global_m", max_dist_current_global_and_prev_global_m_, (float)1.5);  

        nodePrivate.param("max_dist_obstacle_collision", max_dist_obstacle_collision_, (float)0.2);

        nodePrivate.param<string>("base_frame", base_frame_, string("base_footprint"));            

        nodePrivate.param<string>("scan_topic", scan_topic_, string("scan_filtered")); 
        
        nodePrivate.param<string>("cmd_vel_topic", cmd_vel_topic_, string("cmd_vel")); 

        depth_image_sub_.subscribe(nodeHandler_, "/camera/aligned_depth_to_color/image_raw", 1);


        depth_image_sub_.registerCallback(&CogniteamPersonFollower::depthCallback, this);

        info_sub_.subscribe(nodeHandler_, "/camera/aligned_depth_to_color/camera_info", 1);
        info_sub_.registerCallback(&CogniteamPersonFollower::cameraInfoCallback, this);

        imgBgrSubscriber_ = nodeHandler_.subscribe("/camera/color/image_raw", 1,
                                                   &CogniteamPersonFollower::imageCallback, this);

        detected_leg_clusters_sub_ = nodeHandler_.subscribe("detected_leg_clusters", 1,
                                                            &CogniteamPersonFollower::detectedLegCallback, this);

        laserScanSubscriber_ = nodeHandler_.subscribe(scan_topic_, 1,
                                                      &CogniteamPersonFollower::scanCallback, this);

        global_map_sub_ =
            nodeHandler_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                     &CogniteamPersonFollower::globalMapCallback, this);
       

        // publishers

        targets_marker_pub_ = nodeHandler_.advertise<visualization_msgs::MarkerArray>("/camera_targets", 10);

        global_target_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/global_target_marker", 10);

        lidar_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/lidar_target", 10);

        filtered_legs_pub_ = nodeHandler_.advertise<visualization_msgs::MarkerArray>("/filtered_legs", 10);

        lidar_obstacles_pub_ = nodeHandler_.advertise<visualization_msgs::MarkerArray>("/lidar_obstacles", 10);

        is_person_detected_pub_ = nodeHandler_.advertise<std_msgs::Bool>("/is_person_detected", 10);




        twistCommandPublisher_ = 
            nodeHandler_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1, false);

        lidar_fov_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/lidar_area", 10);

        tracking_area_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/follow_area", 10);

        rotation_area_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/rotation_area", 10);

        target_rectangle_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/target_rectangle", 10);
        
        state_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/state_marker", 10);

        image_transport::ImageTransport it(nodeHandler_);
        debug_depth_pub_ = it.advertise("/debug_depth", 1);
        nodeHandler_.param("/debug_depth/compressed/jpeg_quality", 20);

        detection_pub_ = it.advertise("/detection_img", 1);

        debug_map_img_pub_ = it.advertise("/debug_map", 1);

        ///ros params

          

        detectnetWrapper.LoadModel(); 

        cout<<"Loaded models"<<endl;

    }

    ~CogniteamPersonFollower() {}

    void run()
    {   
        ros::NodeHandle nodePrivate("~");

        std::vector<Person> currentCameraTargets;

        while (ros::ok())
        {   
           
            // Check for ros interupts
            ros::spinOnce();

            if (!cameraInfoInited_ || !bgrInited_)
            {
                cerr << " camera info NULL" << endl;
                continue;
            }

            if (!currentBgrImg_.data || !curretDepthImg_.data)
            {
                cerr << " imgs data NULL" << endl;
                continue;
            }

            bgrWorkImg = currentBgrImg_.clone();
            depthImg = curretDepthImg_.clone();

            status_ = "RUNNING";
            nodePrivate.setParam("/person_follower/status", "RUNNING");


            nodePrivate.getParam("/person_follower/person_follower_set_enable", setEnable_);
	        cerr <<"setEnable "<< setEnable_ << endl;

            if(setEnable_ == false) {
                nodePrivate.setParam("/person_follower/status", "STOPPED");
                status_ = "STOPPED";
                state_ = STOP;
                depth_image_sub_.unsubscribe();

            } else {
                nodePrivate.setParam("/person_follower/status", "RUNNING");
                // subsribe to depth
                if(depth_image_sub_.getSubscriber()==NULL) {
                    depth_image_sub_.subscribe(nodeHandler_, "/camera/aligned_depth_to_color/image_raw", 1);
                }
            }

            cerr << getState() << endl;

            switch (state_)
            {
                case STOP:
                {   

                    nodePrivate.getParam("/person_follower/person_follower_set_enable", setEnable_);
                    cerr <<"setEnable "<< setEnable_ << endl;
                    if( setEnable_ == true ) {
                        cerr<<" yakirrrrr: setEnable_ is true "<<endl;    
                        
                        state_ = IDLE;

                        break;
                    
                    } else {

                        cerr<<" yakirrrrr: setEnable_ is false "<<endl;   
                    }

                    

                    // detect with dnn perosns
                    sensor_msgs::ImagePtr image_msg_detect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgrWorkImg).toImageMsg();
                    detectnetWrapper.Detect(image_msg_detect);             

                    // loop over detection and create targets array of persosn
                    for (auto detection : detectnetWrapper.detection2DArray.detections)
                    {    

                        float score = detection.results[0].score;
                        if( score < PERSON_THRESHOLD){
                            continue;
                        }
                        if ((detection.results[0].id == PERSON_CLASS_ID) )
                        {   
                            publishPersonDetected();

                            state_ = STOP;
                            break;
                        }
                    }

                    state_ = STOP;
                    break;

                }
                case IDLE:
                {   


                    currentCameraTargets = collectCameraTargets(bgrWorkImg, depthImg);                   
                
                    if (currentCameraTargets.size() == 0)
                    {

                        cerr << " no depth currentCameraTargets IDLE " << endl;
                        sendStopCmd();
                        state_ = IDLE;
                        break;
                    }                

                    // found target, init global target
                    if (!initGlobalTarget(currentCameraTargets, globalTarget, bgrWorkImg))
                    {
                        // cerr << "iiiiiiiinit failed !!!! " << endl;
                        sendStopCmd();
                        state_ = IDLE;
                        break;
                    }
                    bool isBlocked = isBlockedByObstacle(globalTarget, max_dist_obstacle_collision_);

                    if (isBlocked)
                    {   
                        sendStopCmd();
                        state_ = FOLLOW;
                        break;
                        // sendCmdVel(globalTarget, 1.0, true); 
                        cout<<"Blocked by an obstacle when in IDLE"<<endl;        
                    }
                    else {
                        sendCmdVel(globalTarget);
                    }

                    publishRectangleTargetMarker(globalTarget.distanceM_);
                    
                    state_ = FOLLOW;
                    break;
                }
                case FOLLOW:
                {        
                    currentCameraTargets = collectCameraTargets(bgrWorkImg, depthImg);
                    cerr<<"1111111111111111111111111111111 "<<endl;
                    /// there is no targets
                    if (currentCameraTargets.size() == 0)
                    {
                        cerr<<"2222222222222222222222222222222222222 "<<endl;

                        cerr << " no depth currentCameraTargets FOLLOW " << endl;
                    
                        state_ = FIND_LOST_CAMERA_TARGET_BY_LIDAR;

                        break;
                    }

                    cerr<<"33333333333333333333333333333333333333333 "<<endl;

                
                    
                    // found targets, try to update the global target 
                    if (!updateGlobalTarget(currentCameraTargets, globalTarget))
                    {

                        cerr << "ffffffffffffffffffaild update in FOLLOW" << endl;           

                    

                        state_ = FIND_LOST_CAMERA_TARGET_BY_LIDAR;

                        break;
                    }

                    bool isBlocked = isBlockedByObstacle(globalTarget, max_dist_obstacle_collision_);

                    if (isBlocked)
                    {   
                        sendStopCmd();
                        
                        state_ = FOLLOW;
                        
                        break;
                        // sendCmdVel(globalTarget, 1.0, true);
                    
                    } else {
                        sendCmdVel(globalTarget);
                    }

                    // there is targets, try to inut global target


                    publishRectangleTargetMarker(globalTarget.distanceM_);             


                    state_ = FOLLOW;
                    break;
                }
                case FIND_LOST_CAMERA_TARGET_BY_LIDAR:
                {   

                    cerr<<" inside FIND_LOST_CAMERA_TARGET_BY_LIDAR "<<endl;

                    countFindWithLidar_++;

                    if (updateGlobalTargetBylidar(globalTarget))
                    {   

                        bool isBlocked = isBlockedByObstacle(globalTarget, max_dist_obstacle_collision_);

                        if(isBlocked) {
                            
                            sendStopCmd();
                            break;
                            // sendStopCmd();
                            // state_ = FOLLOW;
                            // break;

                        } else {
                            sendCmdVel(globalTarget);
                        }

                        publishLidarMarker(globalTarget);

                        publishRectangleTargetMarker(globalTarget.distanceM_);

                        state_ = FOLLOW;
                        break;

                    } else {

                        cerr<<" failed to track with lidar !!!!!!!!!!!!!!!!!!!!!111 "<<endl;
                    }
                    // leg target not work, give another chance
                    if (countFindWithLidar_ > maxLlidarTrials_)
                    {
                        countFindWithLidar_ = 0;

                        state_ = IDLE;
                        break;
                    }
                    else
                    {
                        state_ = FIND_LOST_CAMERA_TARGET_BY_LIDAR;
                        break;
                    }
                }
            }

            publishLidarSearchMarker();

            publishTrackingAreaMarker();

            publishRotationAreaMarker();

            publishStateMarker();

            publishFilteredLegs();

            publishTargetsMarkers(currentCameraTargets);

            publishGlobalTargetMarker(globalTarget);

            publishDepthImg();
        }
    }

private:

    void publishPersonDetected(){

        std_msgs::Bool msg;
        msg.data = true;
        is_person_detected_pub_.publish(msg);

    }

    void writeDepthImg()
    {

        if (curretDepthImg_.data)
        {

            Mat depthGrayscale;

            double minVal;
            double maxVal;
            minMaxLoc(depthImg, &minVal, &maxVal);
            depthImg.convertTo(depthGrayscale, CV_8UC1, (255 / (maxVal - minVal)));
            cvtColor(depthGrayscale, depthGrayscale, COLOR_GRAY2BGR);

            cv::Point centerD = globalTarget.centerMassPix_;

            cv::circle(depthGrayscale, cv::Point(depthGrayscale.cols / 2, depthGrayscale.rows / 2),
                       10, cv::Scalar(255, 255, 0), -1);

            cv::rectangle(depthGrayscale, globalTarget.box_, cv::Scalar(255, 0, 0), 5);
            cv::circle(depthGrayscale, centerD, 10, cv::Scalar(0, 0, 255), -1);

            putText(depthGrayscale, getState(), cv::Point(100, 100), 2, 1, cv::Scalar(0, 0, 255));

            putText(depthGrayscale, "angle :" + to_string(globalTarget.degAngle_), cv::Point(100, 500), 5, 1, cv::Scalar(0, 0, 255));

            putText(depthGrayscale, "disntance :" + to_string(globalTarget.distanceM_), cv::Point(100, 700), 5, 1, cv::Scalar(0, 0, 255));

            imwrite("/home/algo-kobuki/depth_imgs/" + to_string(globalCountD_) + ".jpg", depthGrayscale);

            globalCountD_++;
        }
    }

    void publishDetections()
    {

        if (currentBgrImg_.data)
        {

            Mat detectionImg = currentBgrImg_.clone();


            cv::rectangle(detectionImg, globalTarget.box_, cv::Scalar(0, 255, 0), 5);

            cv::circle(detectionImg, globalTarget.centerMassPix_, 10,  cv::Scalar(0, 255, 0), -1);

            putText(detectionImg, getState(), cv::Point(100, 100), 2, 3 ,cv::Scalar(0, 0, 255));

            putText(detectionImg, "angle: " + to_string(globalTarget.degAngle_), cv::Point(100, 500), 2, 3, cv::Scalar(255, 255, 255));

            putText(detectionImg, "distance: " + to_string(globalTarget.distanceM_), cv::Point(100, 700),2,3, 
                cv::Scalar(255, 255, 255));

            auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", detectionImg).toImageMsg();
            detection_pub_.publish(msg);
        }
    }

    void publishDepthImg()
    {

        if (curretDepthImg_.data)
        {

            Mat depthGrayscale;

            double minVal;
            double maxVal;
            minMaxLoc(depthImg, &minVal, &maxVal);
            depthImg.convertTo(depthGrayscale, CV_8UC1, (255 / (maxVal - minVal)));
            cvtColor(depthGrayscale, depthGrayscale, COLOR_GRAY2BGR);

            cv::Point centerD = globalTarget.centerMassPix_;

            cv::circle(depthGrayscale, cv::Point(depthGrayscale.cols / 2, depthGrayscale.rows / 2),
                       10, cv::Scalar(255, 255, 0), -1);

            cv::rectangle(depthGrayscale, globalTarget.box_, cv::Scalar(255, 0, 0), 5);
            cv::circle(depthGrayscale, centerD, 10, cv::Scalar(0, 0, 255), -1);

            putText(depthGrayscale, getState(), cv::Point(100, 100), 2, 1, cv::Scalar(0, 0, 255));

            putText(depthGrayscale, "angle :" + to_string(globalTarget.degAngle_), cv::Point(100, 500), 5, 1, cv::Scalar(0, 0, 255));

            putText(depthGrayscale, "disntance :" + to_string(globalTarget.distanceM_), cv::Point(100, 700), 5, 1, cv::Scalar(0, 0, 255));

            auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", depthGrayscale).toImageMsg();
            debug_depth_pub_.publish(msg);
        }
    }

    string getState()
    {

        switch (state_)
        {
        case IDLE:
        {
            return "IDLE";
        }
        case FOLLOW:
        {
            return "FOLLOW";
        }
        case FIND_LOST_CAMERA_TARGET_BY_LIDAR:
        {
            return "FIND_LOST_CAMERA_TARGET_BY_LIDAR";
        }
        case PREDICT_LOST_TARGET:
        {
            return "PREDICT_LOST_TARGET";
        }
        case STOP:
        {
            return "STOP";
        }
        }

        return "";
    }

    void sendStopCmd()
    {   

        if( follow_without_drive_){
            return;
        }

        geometry_msgs::Twist command;
        command.linear.x = 0;
        command.angular.z = 0;

        twistCommandPublisher_.publish(command);
    }

    bool initGlobalTarget(const std::vector<Person> &persons, Person &globalTarget, 
        const cv::Mat &bgrImg)
    {

        float minDist = std::numeric_limits<float>::max();
        int index = -1;

        cv::Point2d pRef(globalTarget.location_.point.x, globalTarget.location_.point.y);

        // take the closest target to the robot

        for (int i = 0; i < persons.size(); i++)
        {

            float distFromRobot = persons[i].distanceM_;

            float absAngle = persons[i].degAngle_;

            if( !(persons[i].box_.x > 0) ){

                continue;
            }

            if( !(distFromRobot < max_distance_to_follow_) ){

                continue;
            }

            if (distFromRobot < minDist)
            {

                minDist = distFromRobot;
                index = i;
            }
        }

        if (index != -1)
        {

            globalTarget.box_ = persons[index].box_;
            globalTarget.centerMassPix_ = persons[index].centerMassPix_;
            globalTarget.distanceM_ = persons[index].distanceM_;
            globalTarget.location_ = persons[index].location_;
            globalTarget.degAngle_ = persons[index].degAngle_;            

            // if( detectFace(bgrImg, globalTarget.box_)){

            //     return true;
            // }

            // writeDepthImg();

            return true;
        }

        return false;
    }


    bool detectFace(const Mat& img, const cv::Rect& personBbox){
        

        return true;        

    }

    float normalizeLinearSpeed(double distance)
    {

        double curr_val = distance;

        double old_min = targetOffset_ + robotRadius_;
        double old_max = max_distance_to_follow_;

        double new_min = min_linear_speed_;
        double new_max = max_linear_speed_;

        double normSpeed = ((curr_val - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min;

        return normSpeed;
    }

    float normalizeAngularSpeed(double degAngle)
    {

        double curr_val = degAngle;

        double old_min = searchDegMin_;
        double old_max = searchDegMax_;

        double new_min = max_angular_speed_;
        double new_max = min_angular_speed_;

        double normSpeed = ((curr_val - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min;

        return normSpeed;
    }

    void sendCmdVel(const Person &globalTarget, float wAngular = 1.0, bool onlyRotation = false)
    {
        if(onlyRotation == true) {
            cout<<"ROBOT IS BLOCKED BY AN OBSTACLE"<<endl;
        }
        geometry_msgs::Twist command;
        command.linear.x = 0;
        command.angular.z = 0;

        if(follow_without_drive_) {
            return;
        }

        // calculate linear vel
        command.linear.x = normalizeLinearSpeed(globalTarget.distanceM_);

        // calcuale angular vel
        if (globalTarget.degAngle_ > 0)
        {
            // turn left
            command.angular.z = 1 * normalizeAngularSpeed(globalTarget.degAngle_) * wAngular;

            if( globalTarget.distanceM_ > 0)
                command.angular.z /= globalTarget.distanceM_;
        }
        else
        {
            // turn right
            command.angular.z = -1 * normalizeAngularSpeed(globalTarget.degAngle_) * wAngular;

            if( globalTarget.distanceM_ > 0)
                command.angular.z /= globalTarget.distanceM_;
        }

        // if needs only rotate-in-place, or too close to the target
        if (globalTarget.distanceM_ < maxDistanceRotateInPlace_ || globalTarget.distanceM_ < (targetOffset_ + robotRadius_))
        {

            command.linear.x = 0;
        }

        // if angle to small for roatation
        if (fabs(globalTarget.degAngle_) < angleNotRotate_)
        {
            command.angular.z = 0;
        }
        if (onlyRotation)
        {
            command.linear.x = 0;
            cout<<"sending command.linear.x because only Rotation is true"<<endl;
        }

        twistCommandPublisher_.publish(command);
    }

    bool updateGlobalTarget(const vector<Person> &persons, Person &globalTarget)
    {
        float minDist = std::numeric_limits<float>::max();
        int index = -1;

        cv::Point2d prevGlobalLocation(globalTarget.location_.point.x, globalTarget.location_.point.y);

        for (int i = 0; i < persons.size(); i++)
        {

            cv::Point2d pNew(persons[i].location_.point.x, persons[i].location_.point.y);

            float dist = distanceCalculate(prevGlobalLocation, pNew);

            // too far from prev global target
            if (!(dist < max_dist_current_global_and_prev_global_m_))
            {
                cout<<"Return false from updateGlobalTarget because dist is bigger than dist between current and prev target"<<endl;
                cout<<"The distance between the prev target and current target is: "<<dist<<endl;
                cout<<"GLOBAL TARGET X: "<<globalTarget.location_.point.x<<"GLOBAL TARGET Y: "<<globalTarget.location_.point.y<<endl;
                cout<<"CURRENT POINT X: "<<persons[i].location_.point.x<<"CURRENT POINT Y: "<<persons[i].location_.point.y<<endl;
                continue;
            }

            if (dist < minDist)
            {
                minDist = dist;
                index = i;
            }
        }

        // found the global target
        if (index != -1)
        {

            globalTarget.box_ = persons[index].box_;
            globalTarget.centerMassPix_ = persons[index].centerMassPix_;
            globalTarget.distanceM_ = persons[index].distanceM_;
            globalTarget.location_ = persons[index].location_;
            globalTarget.degAngle_ = persons[index].degAngle_;

            // writeDepthImg();

            return true;
        }

        return false;
    }

    float distanceCalculate(cv::Point2d p1, cv::Point2d p2)
    {
        double x = p1.x - p2.x; // calculating number to square in next step
        double y = p1.y - p2.y;
        double dist;

        dist = pow(x, 2) + pow(y, 2); // calculating Euclidean distance
        dist = sqrt(dist);

        return dist;
    }

   

    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {

        globalMap_ = cv::Mat(msg->info.height, msg->info.width, CV_8SC1, Scalar(UNKNOWN));
        memcpy(globalMap_.data, msg->data.data(), msg->info.height * msg->info.width);

        globalMapWidth_ = msg->info.width;
        globalMapHeight_ = msg->info.height;
        mapResolution_ =   msg->info.resolution;
        map_origin_position_x =  msg->info.origin.position.x;
        map_origin_position_y =  msg->info.origin.position.y;



        Mat gray = occupancyGridMatToGrayScale(globalMap_);
        costMap_ = cv::Mat(gray.rows, gray.cols, CV_8UC1, cv::Scalar(0));
        costMap_.setTo(255, gray == 0); 
        dilate(costMap_, costMap_, Mat(), Point(-1, -1), 4, 1, 1);     


        // imwrite("/home/algo-kobuki/depth_imgs/costMap_.png", costMap_);
        
        // Mat dist = costMap_.clone();
        // normalize(dist, dist, 0, 255.0, NORM_MINMAX);
        // imwrite("/home/algo-kobuki/depth_imgs/distanceTransformMap.png", dist);

       

        initMap_ = true;
    }

    bool isBlockedByObstacle(const Person &globalTarget, float maxDistFromObstacleCollision = 0.5)
    {   

        vector<cv::Point2d> currentLidarObstacleBaseLink;

        visualization_msgs::MarkerArray Markerarr;


        geometry_msgs::Point pLeft;
        pLeft.y = -robotRadius_;
        pLeft.x = 0;
        pLeft.z = 0;

        geometry_msgs::Point pRight;
        pRight.y = robotRadius_;
        pRight.x = 0;
        pRight.z = 0;

        geometry_msgs::Point pTopRight;
        pTopRight.y = pRight.y;
        pTopRight.x = pRight.x + globalTarget.distanceM_;
        pRight.z = 0;

        geometry_msgs::Point pTopLeft;
        pTopLeft.y = pLeft.y;
        pTopLeft.x = pLeft.x + globalTarget.distanceM_;
        pRight.z = 0;

        int count = 0;

        for (int i = 0; i < currentScanPoints_.size(); i++)
        {
            /// check collision
            auto pLaser = currentScanPoints_[i];
            
            if(pLaser.y > pLeft.y
                && pLaser.y < pRight.y
                && pLaser.x > pLeft.x 
                && pLaser.x < pTopLeft.x) {
                
                // the obstacle inside the area, check distance
                float distFromRbot = distanceCalculate(pLaser, cv::Point2d(0,0));

                //cout<<"IN isBlockedByObstacle before comparing the current distance: "<<distFromRbot<<" with the max"<<endl;
                
                if( distFromRbot <= maxDistFromObstacleCollision && distFromRbot > 0.3 ) {
                    cerr<<"Detected Obstacle"<<endl;

                    currentLidarObstacleBaseLink.push_back(pLaser);

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = base_frame_;
                    marker.header.stamp = ros::Time::now();
                    marker.ns = "lidar_obstacles";
                    marker.id = count;

                    marker.lifetime = ros::Duration(0.1);

                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.pose.position.x = pLaser.x;
                    marker.pose.position.y = pLaser.y;
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0.0;
                    marker.pose.orientation.y = 0.0;
                    marker.pose.orientation.z = 0.0;
                    marker.pose.orientation.w = 1.0;
                    marker.scale.x = 0.3; 
                    marker.scale.y = 0.3; 
                    marker.scale.z = 0.3;
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 0.411;
                    marker.color.b = 0.705;

                    Markerarr.markers.push_back(marker);

                    lidar_obstacles_pub_.publish(Markerarr);

                    return true;
                }

                // count++;
            }
        }
        return false;
    }

    bool personsFound(const cv::Mat &bgrImg)
    {

        std::vector<Person> currentTargets;

        sensor_msgs::ImagePtr image_msg_detect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgrImg).toImageMsg();
        detectnetWrapper.Detect(image_msg_detect);

        for (auto detection : detectnetWrapper.detection2DArray.detections)
        {
            if ((detection.results[0].id == PERSON_CLASS_ID) & (detection.results[0].score >= PERSON_THRESHOLD))
            {
                return true;
            }
        }

        return false;
    }
    std::vector<Person> collectCameraTargets(const cv::Mat &bgrImg, const cv::Mat &depthImg)
    {

        std::vector<Person> currentTargets;

        // detect with dnn perosns
        sensor_msgs::ImagePtr image_msg_detect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgrImg).toImageMsg();
        detectnetWrapper.Detect(image_msg_detect);

        if (detectnetWrapper.detection2DArray.detections.size() == 0)
        {

            cerr << "No RGB targets" << endl;
            return currentTargets;
        } 

        publishPersonDetected();

        // loop over detection and create targets array of persosn
        for (auto detection : detectnetWrapper.detection2DArray.detections)
        {    

            float score = detection.results[0].score;
            if( score < PERSON_THRESHOLD){
                continue;
            }
            if ((detection.results[0].id == PERSON_CLASS_ID) )
            {

                Person person;
                person.box_ = cv::Rect2d(static_cast<int>(detection.bbox.center.x - (detection.bbox.size_x / 2)),
                                         static_cast<int>(detection.bbox.center.y - (detection.bbox.size_y / 2)),
                                         static_cast<int>(detection.bbox.size_x), static_cast<int>(detection.bbox.size_y));

                person.distanceM_ = getDistanceByBbox(person.box_, person.location_,
                                                      person.centerMassPix_, depthImg);
                if (person.distanceM_ == -1)
                {
                    continue;
                }

                if (!(person.distanceM_ < max_distance_to_follow_))
                {
                    continue;
                }                
                person.degAngle_ =
                    (angles::to_degrees(atan2(person.location_.point.y, person.location_.point.x)));
                

                currentTargets.push_back(person);
            } 
        }

        return currentTargets;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &cam_info)
    {

        if (cameraInfoInited_ == false)
        {
            if (pinholeCameraModel_.fromCameraInfo(*cam_info))
            {
                cameraInfoInited_ = true;
            }
        }
    }

    float getDistanceByCenter(int x, int y, const Mat &depthImg)
    {

        if (cameraInfoInited_)
        {

            cv::Point2d centerObject(x, y);

            float d = depthImg.at<float>(centerObject.y, centerObject.x) / 1000.0; /// IN METERS

            return d;
        }

        return -1;
    }

    float getDistanceByBbox(const cv::Rect &r, geometry_msgs::PointStamped &global_location,
                            cv::Point2d &centerOfMasPix, const Mat &depthImg)
    {

        float global_distance = -1;
        float global_score = std::numeric_limits<float>::max();

        bool found = false;

        cv::Point2d cneterBbox(r.x + (r.width / 2), r.y + (r.height / 2));

        if (cameraInfoInited_)
        {

            int row = r.y + (r.height / 2);

            for (int col = r.x; col < r.x + r.width; col++)
            {

                float distance = float(depthImg.at<float>(row, col)) / 1000.0; //meters

                if (!std::isfinite(distance) || distance > max_distance_to_follow_ || distance <= 0.0)
                {
                    continue;
                }

                auto pix = cv::Point2d(col, row);

                float score = distance * fabs(cneterBbox.x - pix.x);

                if (score < global_score)
                {

                    global_distance = distance;
                    global_score = score;
                    centerOfMasPix = pix;

                    found = true;
                }
            }

            if (found)
            {
                /// convert center of mass tho 3d location
                cv::Point3d threedp = pinholeCameraModel_.projectPixelTo3dRay(centerOfMasPix);
                threedp.x *= global_distance;   
                threedp.y *= global_distance;
                threedp.z *= global_distance;

                global_location = transformDepthBaseLink(threedp); //basek link

                return global_distance;
            } else {

                cerr<<" didnt found good pixel with depth "<<endl;
            }

            return -1;
        }

        return -1;
    }

    void depthCallback(const sensor_msgs::ImageConstPtr &image)
    {

        if (cameraInfoInited_)
        {

            cv_bridge::CvImagePtr cvBridgePtr =
                cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);

            curretDepthImg_ = cvBridgePtr->image;
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {

        if (!bgrInited_)
        {

            bgrInited_ = true;
        }
        try
        {
            currentBgrImg_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {

        currentScanPoints_.clear();

        for (double i = 0; i < scan->ranges.size(); i++)
        {

            if (isinf(scan->ranges[i]) == false)
            {

                double ray_angle = scan->angle_min + (i * scan->angle_increment);

                cv::Point2d rayPoint((scan->ranges[i] * cos(ray_angle)),
                                     (scan->ranges[i] * sin(ray_angle)));

                // transform to bas-link
                cv::Point3d p = cv::Point3d(rayPoint.x, rayPoint.y, 0);

                auto transformedRayPoint = transformFrames(p, base_frame_, scan->header.frame_id, scan->header.stamp);

                currentScanPoints_.push_back(cv::Point2d(transformedRayPoint.point.x, transformedRayPoint.point.y));
            }
        }
    }

    geometry_msgs::PointStamped transformFrames(
        Point3d objectPoint3d, string target_frame, string source_Frame, ros::Time t)
    {

        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = source_Frame;
        pointStampedIn.header.stamp = t;
        pointStampedIn.point.x = objectPoint3d.x;
        pointStampedIn.point.y = objectPoint3d.y;
        pointStampedIn.point.z = objectPoint3d.z;

        try
        {

            tfListener_.lookupTransform(target_frame, source_Frame,
                                        ros::Time(0), cameraTransform);

            tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut);

            initCameraTransform_ = true;

            return pointStampedOut;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());

            if(  target_frame == map_frame_){
                initMap_ = false;
                cerr<<" map not recieved "<<endl;
            }
        }
    }

    geometry_msgs::PointStamped transformDepthBaseLink(
        Point3d objectPoint3d)
    {


        string target_frame = base_frame_;
        string source_frame = depth_frame_;

        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = source_frame;
        pointStampedIn.header.stamp = ros::Time(0);
        pointStampedIn.point.x = objectPoint3d.x;
        pointStampedIn.point.y = objectPoint3d.y;
        pointStampedIn.point.z = objectPoint3d.z;

        if (!initCameraTransform_)
        {

            try
            {

                tfListener_.lookupTransform(target_frame, source_frame,
                                            ros::Time(0), cameraTransform);

                tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut);

                initCameraTransform_ = true;

                return pointStampedOut;
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

        else
        {

            tfListener_.transformPoint(target_frame, pointStampedIn, pointStampedOut);

            return pointStampedOut;
        }
    }

    void publishFilteredLegs(){

        if (filteredLegs_.legs.size() == 0){
            return;
        }

        visualization_msgs::MarkerArray Markerarr;
        Markerarr.markers.resize(filteredLegs_.legs.size());

        for (int i = 0; i < filteredLegs_.legs.size(); i++)
        {

            Markerarr.markers[i].header.frame_id = base_frame_;
            Markerarr.markers[i].header.stamp = ros::Time::now();
            Markerarr.markers[i].ns = "points_and_lines";
            Markerarr.markers[i].id = i + 1;

            Markerarr.markers[i].lifetime = ros::Duration(0.1);

            Markerarr.markers[i].type = visualization_msgs::Marker::SPHERE;
            Markerarr.markers[i].pose.position.x = filteredLegs_.legs[i].position.x;
            Markerarr.markers[i].pose.position.y = filteredLegs_.legs[i].position.y;
            Markerarr.markers[i].pose.position.z = 0;
            Markerarr.markers[i].pose.orientation.x = 0.0;
            Markerarr.markers[i].pose.orientation.y = 0.0;
            Markerarr.markers[i].pose.orientation.z = 0.0;
            Markerarr.markers[i].pose.orientation.w = 1.0;
            Markerarr.markers[i].scale.x = 0.3; 
            Markerarr.markers[i].scale.y = 0.3;
            Markerarr.markers[i].scale.z = 0.3;
            Markerarr.markers[i].color.a = 1.0;
            Markerarr.markers[i].color.r = 0.7;
            Markerarr.markers[i].color.g = 0.5;
            Markerarr.markers[i].color.b = 0.4;


        }

        filtered_legs_pub_.publish(Markerarr);

        
    }
    void publishTargetsMarkers(const std::vector<Person>& targets)
    {
        int start  = 1000;
        visualization_msgs::MarkerArray Markerarr;
        Markerarr.markers.resize(targets.size());

        for (int i = 0; i < targets.size(); i++)
        {

            Markerarr.markers[i].header.frame_id = targets[i].location_.header.frame_id;
            Markerarr.markers[i].header.stamp = ros::Time::now();
            Markerarr.markers[i].ns = "points_and_lines";
            Markerarr.markers[i].id = start + 1;
            Markerarr.markers[i].lifetime = ros::Duration(0.2);
            Markerarr.markers[i].type = visualization_msgs::Marker::SPHERE;
            Markerarr.markers[i].pose.position.x = targets[i].location_.point.x;
            Markerarr.markers[i].pose.position.y = targets[i].location_.point.y;
            Markerarr.markers[i].pose.position.z = 0;
            Markerarr.markers[i].pose.orientation.x = 0.0;
            Markerarr.markers[i].pose.orientation.y = 0.0;
            Markerarr.markers[i].pose.orientation.z = 0.0;
            Markerarr.markers[i].pose.orientation.w = 1.0;
            Markerarr.markers[i].scale.x = 0.3;
            Markerarr.markers[i].scale.y = 0.3;
            Markerarr.markers[i].scale.z = 0.3;
            Markerarr.markers[i].color.a = 1.0;
            Markerarr.markers[i].color.r = 0.2;
            Markerarr.markers[i].color.g = 0.2;
            Markerarr.markers[i].color.b = 0.2;

        }

        if (targets.size() > 0)
            targets_marker_pub_.publish(Markerarr);
    }

    void updateGoalByShift(float x, float y, float shiftM, cv::Point2d &newP)
    {

        float currentDisnace = sqrt(pow(x, 2) + pow(y, 2));

        float diff = currentDisnace - shiftM;

        if (diff <= 0)
        {
            newP = cv::Point2d(x, y);
            return;
        }

        float scalar = diff / currentDisnace;

        newP = cv::Point2d(x * scalar, y * scalar);
    }

    float getRobotHeading(const geometry_msgs::Pose &pose) const
    {

        return atan2((2.0 *
                      (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)),
                     (1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)));
    }

    

    void publishGlobalTargetMarker(const Person &person)
    {


        visualization_msgs::Marker targetFromCameraPoseMsg;
        targetFromCameraPoseMsg.header.frame_id = base_frame_;
        targetFromCameraPoseMsg.header.stamp = ros::Time::now();
        targetFromCameraPoseMsg.ns = "points_and_lines";
        targetFromCameraPoseMsg.id = 777;
        targetFromCameraPoseMsg.type = visualization_msgs::Marker::SPHERE;
        targetFromCameraPoseMsg.lifetime = ros::Duration(0.2);

        targetFromCameraPoseMsg.pose.position.x = person.location_.point.x;
        targetFromCameraPoseMsg.pose.position.y = person.location_.point.y;
        targetFromCameraPoseMsg.pose.position.z = 0;

        targetFromCameraPoseMsg.scale.x = 0.3;
        targetFromCameraPoseMsg.scale.y = 0.3;
        targetFromCameraPoseMsg.scale.z = 0.3;
        targetFromCameraPoseMsg.color.a = 1.0;
        targetFromCameraPoseMsg.color.r = 0.0;
        targetFromCameraPoseMsg.color.g = 1.0;
        targetFromCameraPoseMsg.color.b = 0.0;

        global_target_marker_pub_.publish(targetFromCameraPoseMsg);
    }

     

    void publishLidarMarker(const Person &person)
    {
        visualization_msgs::Marker targetFromLidarPoseMsg;
        targetFromLidarPoseMsg.header.frame_id = base_frame_;
        targetFromLidarPoseMsg.header.stamp = ros::Time::now();
        targetFromLidarPoseMsg.ns = "points_and_lines";
        targetFromLidarPoseMsg.id = 999;
        targetFromLidarPoseMsg.type = visualization_msgs::Marker::SPHERE;
        targetFromLidarPoseMsg.lifetime = ros::Duration(0.2);

        targetFromLidarPoseMsg.pose.position.x = person.location_.point.x;
        targetFromLidarPoseMsg.pose.position.y = person.location_.point.y;
        targetFromLidarPoseMsg.pose.position.z = 0;

        targetFromLidarPoseMsg.scale.x = 0.3;
        targetFromLidarPoseMsg.scale.y = 0.3;
        targetFromLidarPoseMsg.scale.z = 0.3;
        targetFromLidarPoseMsg.color.a = 1.0;
        targetFromLidarPoseMsg.color.r = 1.0;
        targetFromLidarPoseMsg.color.g = 0.0;
        targetFromLidarPoseMsg.color.b = 0.0;

        lidar_marker_pub_.publish(targetFromLidarPoseMsg);
    }

    bool updateGlobalTargetBylidar(Person &globalTarget)
    {

        float minDist = std::numeric_limits<float>::max();
        float index = -1;

        cerr<<" filteredLegs_.legs.size() "<<filteredLegs_.legs.size()<<endl;

        for (int i = 0; i < filteredLegs_.legs.size(); i++)
        {

            float distFromLostTarget = distanceCalculate(cv::Point2d(filteredLegs_.legs[i].position.x,
                                                                     filteredLegs_.legs[i].position.y),
                                                         cv::Point2d(globalTarget.location_.point.x, 
                                                                    globalTarget.location_.point.y));

            if (distFromLostTarget < minDist && 
                distFromLostTarget < maxDistLeg2LostTarget_)
            {
                minDist = distFromLostTarget;
                index = i;
            }
        }

        if (index != -1)
        {

            globalTarget.location_.point.x = filteredLegs_.legs[index].position.x;
            globalTarget.location_.point.y = filteredLegs_.legs[index].position.y;
            globalTarget.location_.point.z = 0;

            globalTarget.distanceM_ = // calc distance from te robot
                distanceCalculate(cv::Point2d(filteredLegs_.legs[index].position.x,
                                              filteredLegs_.legs[index].position.y),
                                  cv::Point2d(0, 0));

            globalTarget.degAngle_ =
                (angles::to_degrees(atan2(filteredLegs_.legs[index].position.y,
                                          filteredLegs_.legs[index].position.x)));

            return true;
        }

        return false;
    }

    cv::Point convertPoseMatToPixMap(const geometry_msgs::PoseStamped &pose) {

        float xPix = (pose.pose.position.x - map_origin_position_x) / mapResolution_;
        float yPix = (pose.pose.position.y - map_origin_position_y) / mapResolution_;

        cv::Point p = cv::Point(xPix, yPix);

        return p;
    }


    void detectedLegCallback(const leg_tracker::LegArrayPtr &msg)
    {
        if ( !initMap_ ){

            return;
        }

        filteredLegs_.legs.clear();
       

        // get filtered legs (BASE-LINK FRAME!!!)
        for (int i = 0; i < msg->legs.size(); i++)
        {

            leg_tracker::Leg leg = msg->legs[i];

          

            bool isObstacle = false;
            // leg is not static obstacle
            if (costMap_.data && initMap_)
            {
                //convert the leg to map frame
                auto legPositionMap = transformFrames(cv::Point3d(leg.position.x, leg.position.y, 0),
                                                     map_frame_, base_frame_, msg->header.stamp);

                geometry_msgs::PoseStamped leg_pose;   
                leg_pose.pose.position.x =  legPositionMap.point.x;
                leg_pose.pose.position.y =  legPositionMap.point.y;
                leg_pose.pose.position.z =  legPositionMap.point.z;

                // convert the leg to pix
                cv::Point legPix = convertPoseMatToPixMap(leg_pose);                                      

                int pixDist = costMap_.at<uchar>(legPix.y, legPix.x);
                leg.distobst = pixDist;

                if( pixDist ==  255){
                    isObstacle = true;
                }
                
            }

            if( isObstacle){
                continue;
            }

            // the leg is good
            float angleFromRobot =
                (angles::to_degrees(atan2(leg.position.y, leg.position.x)));

            float dist = distanceCalculate(cv::Point2d(leg.position.x, leg.position.y),
                                               cv::Point2d(0, 0));

            // leg not too far from the robot (lidar param)
            if (dist < maxDistanceLidarSearch_ && dist > 0.3) // yakir
            {

                filteredLegs_.legs.push_back(leg);
            }
        }

       
    
    }

    void publishTrackingAreaMarker()
    {

        std::srand(std::time(nullptr)); // use current time as seed for random generator

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = base_frame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 6000;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.02;
        line_strip.color.b = 0.5;
        line_strip.color.g = 1.0;
        line_strip.color.r = 0.2;
        line_strip.color.a = 1.0;

        for (uint32_t i = 0; i < 360; ++i)
        {
            geometry_msgs::Point p;

            p.y = (max_distance_to_follow_)*sin(angles::from_degrees(i));
            p.x = (max_distance_to_follow_)*cos(angles::from_degrees(i));
            p.z = 0;

            line_strip.points.push_back(p);
        }

        tracking_area_marker_pub_.publish(line_strip);
    }

    void publishRotationAreaMarker()
    {

        std::srand(std::time(nullptr)); // use current time as seed for random generator

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = base_frame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 6000;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.02;
        line_strip.color.b = 0.2;
        line_strip.color.g = 0.2;
        line_strip.color.r = 1.0;
        line_strip.color.a = 1.0;

        for (uint32_t i = 0; i < 360; ++i)
        {
            geometry_msgs::Point p;

            p.y = (maxDistanceRotateInPlace_)*sin(angles::from_degrees(i));
            p.x = (maxDistanceRotateInPlace_)*cos(angles::from_degrees(i));
            p.z = 0;

            line_strip.points.push_back(p);
        }

        rotation_area_marker_pub_.publish(line_strip);
    }

    void publishStateMarker()
    {

        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.id = 5555;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.text = getState();
        marker.color.b = 1.0;
        marker.color.g = 1.0;
        marker.color.r = 1.0;

        state_marker_pub_.publish(marker);
    }

    Mat occupancyGridMatToGrayScale(const Mat &map)
    {
        // CV_8UC1 - UINT image type
        Mat output(map.rows, map.cols, CV_8UC1, Scalar(205));

        for (int j = 0; j < map.rows; j++)
        {
            for (int i = 0; i < map.cols; i++)
            {

                auto value = map.at<int8_t>(cv::Point(i, j));
                uint8_t newValue = 0;

                if (value == UNKNOWN) // unknown
                    newValue = 205;
                else if (value == FREE) // clear
                    newValue = 254;
                else if (value == BLOCKED) // occupay
                    newValue = 0;

                output.at<uint8_t>(cv::Point(i, j)) = newValue;
            }
        }

        return output;
    }


    void publishRectangleTargetMarker(float distToTargetM) {
        std::srand(std::time(nullptr)); // use current time as seed for random generator
        int random_variable = std::rand();

        visualization_msgs::Marker rectangle;
        rectangle.header.frame_id = base_frame_;
        rectangle.header.stamp = ros::Time::now();
        rectangle.ns = "target_rectangle";
        rectangle.pose.orientation.w = 1.0;
        rectangle.id = 11000;

        rectangle.type = visualization_msgs::Marker::LINE_STRIP;

        rectangle.scale.x = 0.1;
        rectangle.color.b = 0.0;
        rectangle.color.g = 1.0;
        rectangle.color.r = 1.0;

        rectangle.color.a = 1.0;

        geometry_msgs::Point pLeft;
        pLeft.y = -robotRadius_;
        pLeft.x = 0;
        pLeft.z = 0;

        geometry_msgs::Point pRight;
        pRight.y = robotRadius_;
        pRight.x = 0;
        pRight.z = 0;

        geometry_msgs::Point pTopRight;
        pTopRight.y = pRight.y;
        pTopRight.x = pRight.x + distToTargetM;
        pRight.z = 0;

        geometry_msgs::Point pTopLeft;
        pTopLeft.y = pLeft.y;
        pTopLeft.x = pLeft.x + distToTargetM;
        pRight.z = 0;

        rectangle.points.push_back(pLeft);
        rectangle.points.push_back(pRight);
        rectangle.points.push_back(pTopRight);
        rectangle.points.push_back(pTopLeft);
        rectangle.points.push_back(pLeft);

        target_rectangle_pub_.publish(rectangle);
    }

    void publishLidarSearchMarker()
    {

        // std::srand(std::time(nullptr)); // use current time as seed for random generator
        // int random_variable = std::rand();

        // visualization_msgs::Marker line_strip;
        // line_strip.header.frame_id = base_frame_;
        // line_strip.header.stamp = ros::Time::now();
        // line_strip.ns = "points_and_lines";
        // line_strip.pose.orientation.w = 1.0;
        // line_strip.id = 7000;

        // line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        // line_strip.scale.x = 0.02;
        // line_strip.color.b = 1.0;
        // line_strip.color.g = 0.5;
        // line_strip.color.r = 0.7;

        // line_strip.color.a = 1.0;

        // float globalOffset = 0.1;
        // float offset = globalOffset;
        // float delta = 0.1;
        // int num_of_points = 30;

        // for (uint32_t i = 0; i < num_of_points; ++i)
        // {
        //     geometry_msgs::Point p;

        //     p.y = (offset)*sin(angles::from_degrees(searchDegMin_));
        //     p.x = (offset)*cos(angles::from_degrees(searchDegMin_));
        //     p.z = 0;

        //     offset += delta;

        //     if (offset > maxDistanceLidarSearch_)
        //     {
        //         break;
        //     }

        //     line_strip.points.push_back(p);
        // }

        // offset = globalOffset;
        // for (uint32_t i = 0; i < num_of_points; ++i)
        // {
        //     geometry_msgs::Point p;

        //     p.y = (offset)*sin(angles::from_degrees(searchDegMax_));
        //     p.x = (offset)*cos(angles::from_degrees(searchDegMax_));
        //     p.z = 0;

        //     offset += delta;

        //     if (offset > maxDistanceLidarSearch_)
        //     {
        //         break;
        //     }

        //     line_strip.points.push_back(p);
        // }

        // offset = globalOffset;
        // for (uint32_t i = 0; i < num_of_points; ++i)
        // {
        //     geometry_msgs::Point p;

        //     p.y = (offset)*sin(angles::from_degrees(-searchDegMin_));
        //     p.x = (offset)*cos(angles::from_degrees(-searchDegMin_));
        //     p.z = 0;

        //     offset += delta;

        //     if (offset > maxDistanceLidarSearch_)
        //     {
        //         break;
        //     }

        //     line_strip.points.push_back(p);
        // }

        // offset = globalOffset;
        // for (uint32_t i = 0; i < num_of_points; ++i)
        // {
        //     geometry_msgs::Point p;

        //     p.y = (offset)*sin(angles::from_degrees(-searchDegMax_));
        //     p.x = (offset)*cos(angles::from_degrees(-searchDegMax_));
        //     p.z = 0;

        //     offset += delta;

        //     if (offset > maxDistanceLidarSearch_)
        //     {
        //         break;
        //     }

        //     line_strip.points.push_back(p);
        // }

        // lidar_fov_marker_pub_.publish(line_strip);
    }

private:

    ros::NodeHandle nodeHandler_;


    //state and global target
    FollowerState state_ = FollowerState::STOP;
    Person globalTarget;
    string status_;
    bool setEnable_ = false; //stop: false start: true



    string scan_topic_ = "scan_filtered";
    string cmd_vel_topic_ = "cmd_vel";

    // Subscriber
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    ros::Subscriber detected_leg_clusters_sub_;
    ros::Subscriber laserScanSubscriber_;
    ros::Subscriber imgBgrSubscriber_;
    ros::Subscriber global_map_sub_;

    // Publisher
    ros::Publisher is_person_detected_pub_;
    ros::Publisher targets_marker_pub_;
    ros::Publisher global_target_marker_pub_;
    ros::Publisher twistCommandPublisher_;
    ros::Publisher lidar_marker_pub_;
    ros::Publisher lidar_fov_marker_pub_;
    ros::Publisher rotation_area_marker_pub_;
    ros::Publisher tracking_area_marker_pub_;
    ros::Publisher state_marker_pub_;
    ros::Publisher filtered_legs_pub_;
    ros::Publisher lidar_obstacles_pub_;
    ros::Publisher target_rectangle_pub_;
    image_transport::Publisher debug_depth_pub_;
    image_transport::Publisher detection_pub_;

    image_transport::Publisher debug_map_img_pub_;


    // face detection
    // Ptr<Facemark> facemark_;
    // CascadeClassifier face_cascade_;




    // images section
    bool cameraInfoInited_ = false;
    bool bgrInited_ = false;
    cv::Mat curretDepthImg_;
    cv::Mat currentBgrImg_;
    tf::TransformListener tfListener_;
    tf::StampedTransform cameraTransform;
    image_geometry::PinholeCameraModel pinholeCameraModel_;
    Mat bgrWorkImg;
    Mat depthImg;
    DetectnetWrapper detectnetWrapper;


    
    // frames section
    string base_frame_ = "base_footprint";
    string depth_frame_ = "camera_depth_optical_frame";
    string odom_frame_ = "odom";
    string map_frame_ = "map";
    bool initCameraTransform_ = false;



    // following section

    float robotRadius_ = 0.3;
    float targetOffset_ = 0.5;
    float angleNotRotate_ = 15; // deg form each side
    float maxDistInitGlobalTarget_ = 2.0;
    float maxDistanceRotateInPlace_ = 1.5;
    float max_linear_speed_ = 0.8;
    float min_linear_speed_ = 0.2;
    float max_angular_speed_ = 1.2;
    float min_angular_speed_ = 0.5;
    float max_dist_current_global_and_prev_global_m_ = 1.5;
    int countFindWithLidar_ = 0;
    int maxLlidarTrials_ = 5;
    float max_distance_to_follow_ = 5.0;
    float max_dist_obstacle_collision_ = 0.5;
    bool follow_without_drive_ = false;


    // map section
    
    cv::Mat  globalMap_;
    bool     initMap_ = false;
    int      globalMapWidth_ ;
    int      globalMapHeight_ ;
    float    mapResolution_;
    float    map_origin_position_x ;
    float    map_origin_position_y ;
    Mat costMap_;


    // lidar section

    leg_tracker::LegArray filteredLegs_;
    float searchDegMin_ = 0;
    float searchDegMax_ = 170;
    float maxDistanceLidarSearch_ = 5.0;
    float maxDistLeg2LostTarget_ = 1.5; //metets
    float nim_leg_confidence_ = 0.1;
    float max_dist_leg_from_obstacle_wall_ = 0.4;
    


    // obstacle section

    vector<cv::Point2d> currentScanPoints_;


    /// tests
    int globalCountD_ = 0;
};

#endif


// auto end = high_resolution_clock::now();
// auto duration = duration_cast<seconds>(end - LastTimeFollowWithCamera);
// if (duration.count() > maxDurationWithoutCameraFollowing_)
// {

//     state_ = IDLE;

//     break;
// }
