#ifndef DETECTNET_WRAPPER_H
#define DETECTNET_WRAPPER_H

#include <jetson-inference/detectNet.h>
#include "ros_compat.h"
#include "image_converter.h"
#include <image_transport/image_transport.h>

class DetectnetWrapper
{
    private:
        ros::NodeHandle *pDetectetNetNodePrivate;
        detectNet* pdetectNet = NULL;
        uint16_t overlay_flags = detectNet::OVERLAY_NONE;
        std::string model_name  = "ssd-mobilenet-v2";
        std::string model_path;
        std::string prototxt_path;
        std::string class_labels_path;
        std::string input_blob  = DETECTNET_DEFAULT_INPUT;
        std::string output_cvg  = DETECTNET_DEFAULT_COVERAGE;
        std::string output_bbox = DETECTNET_DEFAULT_BBOX;
        std::string overlay_str = "box,labels,conf";

        float mean_pixel = 0.0f;
	    float threshold  = DETECTNET_DEFAULT_THRESHOLD;
        std::vector<std::string> class_descriptions;
        std::int16_t num_classes;
        std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
        std::string model_hash_str;
        size_t model_hash;
        std::string class_key;
        ros::Publisher detectionPublisher;
        image_transport::Publisher overlayPublisher;
        Publisher<vision_msgs::VisionInfo> pInfoPublisher = NULL;
        imageConverter* input_cvt   = NULL;
        imageConverter* overlay_cvt = NULL;
        bool PublishOverlay( detectNet::Detection* detections, int numDetections);
        


    public:
        vision_msgs::Detection2DArray detection2DArray;

        DetectnetWrapper();
        ~DetectnetWrapper(){};
        void LoadModel();
        void Detect(sensor_msgs::ImagePtr input);
        void PublishDetections();
};

#endif