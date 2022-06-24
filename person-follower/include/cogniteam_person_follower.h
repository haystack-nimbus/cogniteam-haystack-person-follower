

#ifndef COGNITEAM_PERSON_FOLLOWER_H
#define COGNITEAM_PERSON_FOLLOWER_H

#include <ros/ros.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_ros/cameraTransformlistener.h>
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

#include <image_geometry/stereo_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf/tf.h>
#include <tf/cameraTransformlistener.h>
#include <visualization_msgs/MarkerArray.h>
#include <angles/angles.h>

#include <string>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <chrono>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include "../opencv/cv-helpers.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <tf2_ros/static_cameraTransformbroadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <leg_tracker/LegArray.h>

#include "../include/detectnet.h"

using namespace std;
using namespace cv;
using namespace std::chrono;

#define PERSON_CLASS_ID (1)
#define PERSON_THRESHOLD (0.7)

#define UNKNOWN (-1)
#define FREE (0)
#define BLOCKED (10)0

enum FollowerState
{
    IDLE,
    FOLLOW,
    FIND_LOST_CAMERA_TARGET_BY_LIDAR,
    SCAN

};

struct Person
{

    cv::Rect box_;

    cv::Point2d centerMassPix_;

    float distanceM_ = 0.0;

    geometry_msgs::PointStamped location_;

    float degAngle_ = 0.0;
};

class CogniteamPersonFollower
{

public:
    CogniteamPersonFollower()
    {

        depth_image_sub_.subscribe(nodeHandler_, "/camera/aligned_depth_to_color/image_raw", 1);
        depth_image_sub_.registerCallback(&CogniteamPersonFollower::depthCallback, this);

        info_sub_.subscribe(nodeHandler_, "/camera/aligned_depth_to_color/camera_info", 1);
        info_sub_.registerCallback(&CogniteamPersonFollower::cameraInfoCallback, this);

        imgBgrSubscriber_ = nodeHandler_.subscribe("/camera/color/image_raw", 1,
                                                   &CogniteamPersonFollower::imageCallback, this);

        detected_leg_clusters_sub_ = nodeHandler_.subscribe("detected_leg_clusters", 1,
                                                            &CogniteamPersonFollower::detectedLegCallback, this);

        laserScanSubscriber_ = nodeHandler_.subscribe("/scan", 1,
                                                      &CogniteamPersonFollower::scanCallback, this);

        global_map_sub_ =
            node_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                     &CogniteamPersonFollower::globalMapCallback, this);

        // publishers

        targets_marker_pub_ = nodeHandler_.advertise<visualization_msgs::MarkerArray>("/persons_markers", 10);

        goal_marker_pub_ = nodeHandler_.advertise<visualization_msgs::MarkerArray>("/target_marker", 10);

        lidar_marker_pub_ = nodeHandler_.advertise<visualization_msgs::MarkerArray>("/lidar_target", 10);

        twistCommandPublisher_ = nodeHandler_.advertise<geometry_msgs::Twist>("/raw_cmd_vel/" /*"mobile_base/commands/velocity"*/, 1, false);

        lidar_fov_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/lidar_fov_markers", 10);

        tracking_area_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/tacking_area_markers", 10);

        rotation_area_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/rotation_area_markers", 10);

        state_marker_pub_ = nodeHandler_.advertise<visualization_msgs::Marker>("/state_marker", 10);

        image_transport::ImageTransport it(nodeHandler_);
        debug_depth_pub_ = it.advertise("/debug_depth", 1);
        nodeHandler_.param("/debug_depth/compressed/jpeg_quality", 20);

        detection_pub_ = it.advertise("/detection_img", 1);
        nodeHandler_.param("/detection_img/compressed/jpeg_quality", 20);

        detectnetWrapper.LoadModel();
    }

    ~CogniteamPersonFollower() {}

    void run()
    {
        std::vector<Person> targets;

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

            cerr << getState() << endl;

            auto LastTimeFollowWithCamera = high_resolution_clock::now();

            switch (state_)
            {
            case IDLE:
            {
                targets = collectCameraTargets(bgrWorkImg, depthImg);

                if (targets.size() == 0)
                {

                    cerr << " no depth targets " << endl;
                    sendStopCmd();

                    state_ = IDLE;
                    break;
                }

                // found target, init target
                if (!initGlobalTarget(targets, globalTarget))
                {

                    cerr << "iiiiiiiinit failed !!!! " << endl;

                    sendStopCmd();

                    state_ = IDLE;
                    break;
                }

                bool isBlocked = isBlockedByObstacle(globalTarget);

                if (isBlocked)
                {

                    sendStopCmd();
                    state_ = IDLE;
                    break;
                }

                publishDetections();

                LastTimeFollowWithCamera = high_resolution_clock::now();

                sendCmdVel(globalTarget);

                state_ = FOLLOW;
                break;
            }
            case FOLLOW:
            {

                targets = collectCameraTargets(bgrWorkImg, depthImg);

                /// there is no targets
                if (targets.size() == 0)
                {

                    cerr << "NNNNNNNNNNNNNNNNNNNO TARGETS " << endl;

                    auto end = high_resolution_clock::now();
                    auto duration = duration_cast<seconds>(end - LastTimeFollowWithCamera);

                    if (duration.count() > maxDurationWithoutCameraFollowing_)
                    {

                        state_ = IDLE;

                        break;
                    }

                    state_ = FIND_LOST_CAMERA_TARGET_BY_LIDAR;

                    break;
                }

                if (!updateGlobalTarget(targets, globalTarget))
                {

                    cerr << "ffffffffffffffffffaild update " << endl;

                    auto end = high_resolution_clock::now();
                    auto duration = duration_cast<seconds>(end - LastTimeFollowWithCamera);
                    if (duration.count() > maxDurationWithoutCameraFollowing_)
                    {

                        state_ = IDLE;

                        break;
                    }

                    // it it false, we found global target but it noise,
                    // stay this state but robot stop

                    state_ = FIND_LOST_CAMERA_TARGET_BY_LIDAR;

                    break;
                }

                bool isBlocked = isBlockedByObstacle(globalTarget);

                if (isBlocked)
                {

                    sendStopCmd();

                    state_ = IDLE;

                    break;
                }

                // there is targets, try to inut global target

                publishDetections();

                LastTimeFollowWithCamera = high_resolution_clock::now();
                ;

                sendCmdVel(globalTarget);

                state_ = FOLLOW;
                break;
            }
            case FIND_LOST_CAMERA_TARGET_BY_LIDAR:
            {

                cerr << " last angle and distnace lost target: " << globalTarget.degAngle_ << ", " << globalTarget.distanceM_ << endl;

                countFindWithLidar_++;

                if (updateGlobalTargetBylidar(globalTarget))
                {

                    sendCmdVel(globalTarget, 1.3, true);

                    publishLidarMarker(globalTarget);

                    state_ = FOLLOW;
                    break;
                }

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
            case SCAN:
            {

                state_ = IDLE;
                break;
            }
            }

            publishLidarSearchMarker();

            publishTrackingAreaMarker();

            publishRotationAreaMarker();

            publishStateMarker();

            // publishTargetsMarkers(targets);

            publishGlobalTargetMarker(globalTarget);

            // publishDepthImg();
        }
    }

private:
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

            cv::Point centerD = globalTarget.centerMassPix_;

            cv::circle(detectionImg, cv::Point(detectionImg.cols / 2, detectionImg.rows / 2),
                       10, cv::Scalar(255, 255, 0), -1);

            cv::rectangle(detectionImg, globalTarget.box_, cv::Scalar(0, 255, 0), 5);
            cv::circle(detectionImg, centerD, 10, cv::Scalar(0, 0, 255), -1);

            putText(detectionImg, getState(), cv::Point(100, 100), 2, 1, cv::Scalar(0, 0, 255));

            putText(detectionImg, "angle :" + to_string(globalTarget.degAngle_), cv::Point(100, 500), 5, 1, cv::Scalar(0, 0, 255));

            putText(detectionImg, "disntance :" + to_string(globalTarget.distanceM_), cv::Point(100, 700), 5, 1, cv::Scalar(0, 0, 255));

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
        case SCAN:
        {
            return "SCAN";
        }
        }

        return "";
    }

    void sendStopCmd()
    {

        geometry_msgs::Twist command;
        command.linear.x = 0;
        command.angular.z = 0;

        twistCommandPublisher_.publish(command);
    }

    bool initGlobalTarget(const std::vector<Person> &persons, Person &globalTarget)
    {

        float minDist = 9999;
        int index = -1;

        cv::Point2d pRef(globalTarget.location_.point.x, globalTarget.location_.point.y);

        // take the closest target to the robot

        for (int i = 0; i < persons.size(); i++)
        {

            float distFromRobot = persons[i].distanceM_;

            float absAngle = persons[i].degAngle_;

            if (fabs(absAngle) > 45)
            {
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

            // writeDepthImg();

            return true;
        }

        return false;
    }

    float normalizeLinearSpeed(double distance)
    {

        double curr_val = distance;

        double old_min = targetOffset_;
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

        geometry_msgs::Twist command;
        command.linear.x = 0;
        command.angular.z = 0;

        // calculate linear vel
        command.linear.x = normalizeLinearSpeed(globalTarget.distanceM_);

        // calcuale angular vel
        if (globalTarget.degAngle_ > 0)
        {
            // turn left
            command.angular.z = 1 * normalizeAngularSpeed(globalTarget.degAngle_) * wAngular;
        }
        else
        {
            // turn right
            command.angular.z = -1 * normalizeAngularSpeed(globalTarget.degAngle_) * wAngular;
        }

        // if needs only rotate-in-place, or too close to the target
        if (globalTarget.distanceM_ < maxDistanceRotateInPlace_ || globalTarget.distanceM_ < targetOffset_)
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
        }

        twistCommandPublisher_.publish(command);
    }

    bool updateGlobalTarget(const vector<Person> &persons, Person &globalTarget)
    {

        float minDist = 9999;
        int index = -1;

        cv::Point2d prevGlobalLocation(globalTarget.location_.point.x, globalTarget.location_.point.y);

        for (int i = 0; i < persons.size(); i++)
        {

            cv::Point2d pNew(persons[i].location_.point.x, persons[i].location_.point.x);

            float dist = distanceCalculate(prevGlobalLocation, pNew);

            // too far from prev global target
            if (!(dist < max_dist_current_global_and_prev_global_m_))
            {

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

        initMap_ = true;
    }

    bool isBlockedByObstacle(const Person &globalTarget)
    {

        float m = globalTarget.location_.point.y / globalTarget.location_.point.x;

        /// Y = M * X + N
        float n = globalTarget.location_.point.y - (m * globalTarget.location_.point.x);

        for (int i = 0; i < currentScanPoints_.size(); i++)
        {

            /// check collision

            auto pLaser = currentScanPoints_[i];

            // if ( ( (pLaser.x*m) + n ) == pLaser.y ){

            //     return true;
            // }
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

        sensor_msgs::ImagePtr image_msg_detect = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgrImg).toImageMsg();
        detectnetWrapper.Detect(image_msg_detect);

        if (detectnetWrapper.detection2DArray.detections.size() == 0)
        {

            cerr << " no RGB targets " << endl;
            return currentTargets;
        }

        for (auto detection : detectnetWrapper.detection2DArray.detections)
        {
            if ((detection.results[0].id == PERSON_CLASS_ID) & (detection.results[0].score >= PERSON_THRESHOLD))
            {

                Person person;
                person.box_ = cv::Rect2d(static_cast<int>(detection.bbox.center.x - (detection.bbox.size_x / 2)),
                                         static_cast<int>(detection.bbox.center.y - (detection.bbox.size_y / 2)),
                                         static_cast<int>(detection.bbox.size_x), static_cast<int>(detection.bbox.size_y));

                person.distanceM_ = getDistanceByBbox(person.box_, person.location_,
                                                      person.centerMassPix_, depthImg);

                if (person.distanceM_ == -1)
                {
                    cerr << "111111111111111111111111111111" << endl;
                    continue;
                }

                person.degAngle_ =
                    (angles::to_degrees(atan2(person.location_.point.y, person.location_.point.x)));

                if (!(person.distanceM_ < max_distance_to_follow_))
                {

                    continue;
                }

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
        float global_score = 99999;

        bool found = false;

        cv::Point2d cneterBbox(r.x + (r.width / 2), r.y + (r.height / 2));

        if (cameraInfoInited_)
        {

            int row = r.y + (r.height / 2);

            for (int col = r.x; col < r.x + r.width; col++)
            {

                float distance = float(depthImg.at<float>(row, col)) / 1000.0;

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

                cv::Point3d threedp = pinholeCameraModel_.projectPixelTo3dRay(centerOfMasPix);
                threedp.x *= global_distance;
                threedp.y *= global_distance;
                threedp.z *= global_distance;

                global_location = transformDepthBaseLink(threedp);

                return global_distance;
            }

            return -1;
        }

        return -1;
    }

    // yakir
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

                auto transformedRayPoint = transformFrames(p, base_frame_, scan->header.frame_id);

                currentScanPoints_.push_back(cv::Point2d(transformedRayPoint.point.x, transformedRayPoint.point.y));
            }
        }
    }

    geometry_msgs::PointStamped transformFrames(
        Point3d objectPoint3d, string target_frame, string source_Frame)
    {

        geometry_msgs::PointStamped pointStampedIn;
        geometry_msgs::PointStamped pointStampedOut;

        pointStampedIn.header.frame_id = source_Frame;
        pointStampedIn.header.stamp = ros::Time(0);
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
        }
    }

    geometry_msgs::PointStamped transformDepthBaseLink(
        Point3d objectPoint3d)
    {


        string target_frame = base_frame_;
        string source_frame = base_Frame_;

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

    void publishTargetsMarkers(const std::vector<Person> targets)
    {

        visualization_msgs::MarkerArray Markerarr;
        Markerarr.markers.resize(targets.size());

        for (int i = 0; i < targets.size(); i++)
        {

            Markerarr.markers[i].header.frame_id = targets[i].location_.header.frame_id;
            Markerarr.markers[i].header.stamp = ros::Time::now();
            Markerarr.markers[i].ns = "points_and_lines";
            Markerarr.markers[i].id = i + 1;
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
            Markerarr.markers[i].color.r = 0.0;
            Markerarr.markers[i].color.g = 1.0;
            Markerarr.markers[i].color.b = 0.0;

            // Markerarr.markers[i].lifetime = ros::Duration(1.0);
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

    void calculateGoalHeading(geometry_msgs::PoseStamped &currentGoal)
    {

        // get current robot pose
        geometry_msgs::PoseStamped robotPose;

        try
        {
            tf::StampedTransform transform;
            // get current robot pose
            tfListener_.lookupTransform(base_frame_, "odom",
                                        ros::Time(0), transform);

            robotPose.pose.position.x = transform.getOrigin().x();
            robotPose.pose.position.y = transform.getOrigin().y();
            robotPose.pose.position.z = 0;
            robotPose.pose.orientation.x = transform.getRotation().x();
            robotPose.pose.orientation.y = transform.getRotation().y();
            robotPose.pose.orientation.z = transform.getRotation().z();
            robotPose.pose.orientation.w = transform.getRotation().w();

            float angleFromTarget = atan2(currentGoal.pose.position.y,
                                          currentGoal.pose.position.x);

            tf2::Quaternion orientation;
            orientation.setRPY(0, 0, angleFromTarget);
            cerr << " the angle deg is " << angles::to_degrees(angleFromTarget) << endl;

            // orientation.setRPY( 0, 0, angleFromTarget + angles::from_degrees(180));
            // cerr<<" the angle deg is "<<angles::to_degrees(angleFromTarget + angles::from_degrees(180))<<endl;

            // set the new orientation
            currentGoal.pose.orientation.w = orientation.getW();
            currentGoal.pose.orientation.x = orientation.getX();
            currentGoal.pose.orientation.y = orientation.getY();
            currentGoal.pose.orientation.z = orientation.getZ();

            return;
        }

        catch (...)
        {
            cerr << " error getting robot pose " << endl;
            currentGoal.pose.orientation.w = 1;
            currentGoal.pose.orientation.x = 0;
            currentGoal.pose.orientation.y = 0;
            currentGoal.pose.orientation.z = 0;
            return;
        }
    }

    void publishGlobalTargetMarker(const Person &person)
    {

        visualization_msgs::MarkerArray Markerarr;
        Markerarr.markers.resize(1);

        visualization_msgs::Marker targetFromCameraPoseMsg;
        Markerarr.markers[0].header.frame_id = base_frame_;
        Markerarr.markers[0].header.stamp = ros::Time::now();
        Markerarr.markers[0].ns = "points_and_lines";
        Markerarr.markers[0].id = 1;
        Markerarr.markers[0].type = visualization_msgs::Marker::SPHERE;
        Markerarr.markers[0].lifetime = ros::Duration(0.2);

        Markerarr.markers[0].pose.position.x = person.location_.point.x;
        Markerarr.markers[0].pose.position.y = person.location_.point.y;
        Markerarr.markers[0].pose.position.z = 0;

        Markerarr.markers[0].scale.x = 0.3;
        Markerarr.markers[0].scale.y = 0.3;
        Markerarr.markers[0].scale.z = 0.3;
        Markerarr.markers[0].color.a = 1.0;
        Markerarr.markers[0].color.r = 0.0;
        Markerarr.markers[0].color.g = 1.0;
        Markerarr.markers[0].color.b = 0.0;

        goal_marker_pub_.publish(Markerarr);
    }

    void publishLidarMarker(const Person &person)
    {

        visualization_msgs::MarkerArray Markerarr;
        Markerarr.markers.resize(1);

        visualization_msgs::Marker targetFromCameraPoseMsg;
        Markerarr.markers[0].header.frame_id = base_frame_;
        Markerarr.markers[0].header.stamp = ros::Time::now();
        Markerarr.markers[0].ns = "points_and_lines";
        Markerarr.markers[0].id = 999;
        Markerarr.markers[0].type = visualization_msgs::Marker::SPHERE;
        Markerarr.markers[0].lifetime = ros::Duration(0.2);

        Markerarr.markers[0].pose.position.x = person.location_.point.x;
        Markerarr.markers[0].pose.position.y = person.location_.point.y;
        Markerarr.markers[0].pose.position.z = 0;

        Markerarr.markers[0].scale.x = 0.3;
        Markerarr.markers[0].scale.y = 0.3;
        Markerarr.markers[0].scale.z = 0.3;
        Markerarr.markers[0].color.a = 1.0;
        Markerarr.markers[0].color.r = 1.0;
        Markerarr.markers[0].color.g = 0.0;
        Markerarr.markers[0].color.b = 0.0;

        lidar_marker_pub_.publish(Markerarr);
    }

    bool updateGlobalTargetBylidar(Person &globalTarget)
    {

        float minDist = 9999;
        float index = -1;

        for (int i = 0; i < filteredLegs_.legs.size(); i++)
        {

            float distFromLostTarget = distanceCalculate(cv::Point2d(filteredLegs_.legs[i].position.x,
                                                                     filteredLegs_.legs[i].position.y),
                                                         cv::Point2d(globalTarget.location_.point.x, globalTarget.location_.point.y));

            if (distFromLostTarget < minDist && distFromLostTarget < maxDistLeg2LostTarget_)
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

        filteredLegs_.legs.clear();

        // get filtered legs (BASE-LINK FRAME!!!)
        for (int i = 0; i < msg->legs.size(); i++)
        {

            leg_tracker::Leg leg = msg->legs[i];

            // confidence validation
            if (leg.confidence < 0.1)
            {
                continue;
            }

            // leg is not static obstacle
            if (globalMap_.data && initMap_)
            {

                legPositionMap = transformFrames(cv::Point3d(leg.position.x, leg.position.y, 0),
                                                     map_frame_, base_frame);
                
                cv::Point legPix = convertPoseMatToPixMap(legPositionMap);                                      

                auto value = globalMap_.at<int8_t>(cv::Point(legPix.y, legPix.x));
                uint8_t newValue = 0;

                if (value == BLOCKED){
                    continue;
                }
            }

            float angleFromRobot =
                (angles::to_degrees(atan2(leg.position.y, leg.position.x)));

            // leg inside Boundaries
            if ((angleFromRobot >= searchDegMin_ && angleFromRobot <= searchDegMax_) || (angleFromRobot >= (-searchDegMax_) && angleFromRobot <= (-searchDegMin_)))
            {

                float dist = distanceCalculate(cv::Point2d(leg.position.x, leg.position.y),
                                               cv::Point2d(0, 0));

                // leg not too far

                if (dist < maxDistanceLidarSearch_)
                {

                    filteredLegs_.legs.push_back(leg);
                }
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

    void publishLidarSearchMarker()
    {

        std::srand(std::time(nullptr)); // use current time as seed for random generator
        int random_variable = std::rand();

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = base_frame_;
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "points_and_lines";
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 7000;

        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.02;
        line_strip.color.b = 1.0;
        line_strip.color.g = 0.5;
        line_strip.color.r = 0.7;

        line_strip.color.a = 1.0;

        float globalOffset = 0.1;
        float offset = globalOffset;
        float delta = 0.1;
        int num_of_points = 30;

        for (uint32_t i = 0; i < num_of_points; ++i)
        {
            geometry_msgs::Point p;

            p.y = (offset)*sin(angles::from_degrees(searchDegMin_));
            p.x = (offset)*cos(angles::from_degrees(searchDegMin_));
            p.z = 0;

            offset += delta;

            if (offset > maxDistanceLidarSearch_)
            {
                break;
            }

            line_strip.points.push_back(p);
        }

        offset = globalOffset;
        for (uint32_t i = 0; i < num_of_points; ++i)
        {
            geometry_msgs::Point p;

            p.y = (offset)*sin(angles::from_degrees(searchDegMax_));
            p.x = (offset)*cos(angles::from_degrees(searchDegMax_));
            p.z = 0;

            offset += delta;

            if (offset > maxDistanceLidarSearch_)
            {
                break;
            }

            line_strip.points.push_back(p);
        }

        offset = globalOffset;
        for (uint32_t i = 0; i < num_of_points; ++i)
        {
            geometry_msgs::Point p;

            p.y = (offset)*sin(angles::from_degrees(-searchDegMin_));
            p.x = (offset)*cos(angles::from_degrees(-searchDegMin_));
            p.z = 0;

            offset += delta;

            if (offset > maxDistanceLidarSearch_)
            {
                break;
            }

            line_strip.points.push_back(p);
        }

        offset = globalOffset;
        for (uint32_t i = 0; i < num_of_points; ++i)
        {
            geometry_msgs::Point p;

            p.y = (offset)*sin(angles::from_degrees(-searchDegMax_));
            p.x = (offset)*cos(angles::from_degrees(-searchDegMax_));
            p.z = 0;

            offset += delta;

            if (offset > maxDistanceLidarSearch_)
            {
                break;
            }

            line_strip.points.push_back(p);
        }

        lidar_fov_marker_pub_.publish(line_strip);
    }

private:
    ros::NodeHandle nodeHandler_;

    DetectnetWrapper detectnetWrapper;

    FollowerState state_ = FollowerState::IDLE;

    Person globalTarget;

    image_geometry::PinholeCameraModel pinholeCameraModel_;

    // Subscriber
    message_filters::Subscriber<sensor_msgs::Image> depth_image_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
    ros::Subscriber detected_leg_clusters_sub_;
    ros::Subscriber laserScanSubscriber_;
    ros::Subscriber imgBgrSubscriber_;
    ros::Subscriber global_map_sub_;

    // Publisher
    ros::Publisher targets_marker_pub_;
    ros::Publisher goal_marker_pub_;
    ros::Publisher twistCommandPublisher_;
    ros::Publisher lidar_marker_pub_;
    ros::Publisher lidar_fov_marker_pub_;
    ros::Publisher rotation_area_marker_pub_;
    ros::Publisher tracking_area_marker_pub_;
    ros::Publisher state_marker_pub_;

    image_transport::Publisher debug_depth_pub_;
    image_transport::Publisher detection_pub_;

    bool cameraInfoInited_ = false;
    bool bgrInited_ = false;
    cv::Mat curretDepthImg_;
    cv::Mat currentBgrImg_;
    tf::TransformListener tfListener_;
    tf::StampedTransform cameraTransform;

    Mat bgrWorkImg;
    Mat depthImg;

    bool initCameraTransform_ = false;
  
    string base_frame_ = "base_link";
    string depth_frame_ = "camera_depth_optical_frame";
    string odom_frame_ = "odom";
    string map_frame_ = "map";

    float angleNotRotate_ = 15;

    float maxDistanceRotateInPlace_ = 1.5;

    float targetOffset_ = 0.6;

    float max_linear_speed_ = 2.0;
    float min_linear_speed_ = 0.5;

    float max_angular_speed_ = 1.5;
    float min_angular_speed_ = 1.0;

    float max_distance_to_follow_ = 5.0;

    float max_dist_current_global_and_prev_global_m_ = 0.5;

    int maxDurationWithoutCameraFollowing_ = 5.0; // seconds

    // map section
    cv::Mat  globalMap_;
    bool     initMap_ = false;
    int      globalMapWidth_ ;
    int      globalMapHeight_ ;
    float    mapResolution_;
    float    map_origin_position_x ;
    float    map_origin_position_y ;

    // lidar section

    float searchDegMin_ = 0;
    float searchDegMax_ = 200;
    float maxDistanceLidarSearch_ = maxDistanceRotateInPlace_ * 2;
    float maxDistLeg2LostTarget_ = 1.0;

    int countFindWithLidar_ = 0;
    int maxLlidarTrials_ = 4;

    vector<cv::Point2d> currentScanPoints_;

    leg_tracker::LegArray filteredLegs_;

    /// tests
    int globalCountD_ = 0;
};

#endif
