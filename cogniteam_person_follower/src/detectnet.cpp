#include "../include/detectnet.h"


DetectnetWrapper::DetectnetWrapper()
{
	pDetectetNetNodePrivate = new ros::NodeHandle ("~");
	
	pDetectetNetNodePrivate->setParam("model_name", model_name);
    // ROS_DECLARE_PARAMETER("model_name", model_name);
	pDetectetNetNodePrivate->setParam("model_path", model_path);
	// ROS_DECLARE_PARAMETER("model_path", model_path);
	pDetectetNetNodePrivate->setParam("prototxt_path", prototxt_path);
	// ROS_DECLARE_PARAMETER("prototxt_path", prototxt_path);
	pDetectetNetNodePrivate->setParam("class_labels_path", class_labels_path);
	// ROS_DECLARE_PARAMETER("class_labels_path", class_labels_path);
	pDetectetNetNodePrivate->setParam("input_blob", input_blob);
	// ROS_DECLARE_PARAMETER("input_blob", input_blob);
	pDetectetNetNodePrivate->setParam("output_cvg", output_cvg);
	// ROS_DECLARE_PARAMETER("output_cvg", output_cvg);
	pDetectetNetNodePrivate->setParam("output_bbox", output_bbox);
	// ROS_DECLARE_PARAMETER("output_bbox", output_bbox);
	pDetectetNetNodePrivate->setParam("overlay_flags", overlay_str);
	// ROS_DECLARE_PARAMETER("overlay_flags", overlay_str);
	pDetectetNetNodePrivate->setParam("mean_pixel_value", mean_pixel);
	// ROS_DECLARE_PARAMETER("mean_pixel_value", mean_pixel);
	pDetectetNetNodePrivate->setParam("threshold", threshold);
	// ROS_DECLARE_PARAMETER("threshold", threshold);
	pDetectetNetNodePrivate->setParam("model_name", model_name);


	/*
	 * retrieve parameters
	 */
	// ROS_GET_PARAMETER("model_name", model_name);
	// ROS_GET_PARAMETER("model_path", model_path);
	// ROS_GET_PARAMETER("prototxt_path", prototxt_path);
	// ROS_GET_PARAMETER("class_labels_path", class_labels_path);
	// ROS_GET_PARAMETER("input_blob", input_blob);
	// ROS_GET_PARAMETER("output_cvg", output_cvg);
	// ROS_GET_PARAMETER("output_bbox", output_bbox);
	// ROS_GET_PARAMETER("overlay_flags", overlay_str);
	// ROS_GET_PARAMETER("mean_pixel_value", mean_pixel);
	// ROS_GET_PARAMETER("threshold", threshold);

	pDetectetNetNodePrivate->getParam("model_name", model_name);
	pDetectetNetNodePrivate->getParam("model_path", model_path);
	pDetectetNetNodePrivate->getParam("prototxt_path", prototxt_path);
	pDetectetNetNodePrivate->getParam("class_labels_path", class_labels_path);
	pDetectetNetNodePrivate->getParam("input_blob", input_blob);
	pDetectetNetNodePrivate->getParam("output_cvg", output_cvg);
	pDetectetNetNodePrivate->getParam("output_bbox", output_bbox);
	pDetectetNetNodePrivate->getParam("overlay_flags", overlay_str);
	pDetectetNetNodePrivate->getParam("mean_pixel_value", mean_pixel);
	pDetectetNetNodePrivate->getParam("threshold", threshold);
	pDetectetNetNodePrivate->getParam("model_name", model_name);
	// if(!pDetectetNetNodePrivate->hasParam("model_name"))
	// {
	// 	pDetectetNetNodePrivate->getParam("model_name ", default_value);
	// }
	// else
	// {
	// 	ROS_INFO("Param %s not set", "model_name");
	// }
	image_transport::ImageTransport imageTransporter(*pDetectetNetNodePrivate);
    overlay_flags = detectNet::OverlayFlagsFromStr(overlay_str.c_str());
	input_cvt = new imageConverter();
	overlay_cvt = new imageConverter();
	
	if( !input_cvt )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return;
	}

	detectionPublisher = pDetectetNetNodePrivate->advertise<vision_msgs::Detection2DArray>("detections", 5);
	overlayPublisher = imageTransporter.advertise("detection_overlay", 5);

}


void DetectnetWrapper::LoadModel()
{
    /*
	 * load object detection network
	 */

	if( model_path.size() > 0 )
	{
		// create network using custom model paths
		pdetectNet = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), 
						    mean_pixel, class_labels_path.c_str(), threshold, 
						    input_blob.c_str(), output_cvg.c_str(), output_bbox.c_str());
	}
	else
	{
		// determine which built-in model was requested
		detectNet::NetworkType model = detectNet::NetworkTypeFromStr(model_name.c_str());

		if( model == detectNet::CUSTOM )
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to pednet", model_name.c_str());
			model = detectNet::SSD_MOBILENET_V2;
		}

		// create network using the built-in model
		pdetectNet = detectNet::Create(model, threshold);
	}

	if( !pdetectNet )
	{
		ROS_ERROR("failed to load detectNet model");
		return;
	}
    else
    {
        /*
        * create the class labels parameter vector
        */
        model_hash_str = std::string(pdetectNet->GetModelPath()) + std::string(pdetectNet->GetClassPath());
        model_hash = model_hasher(model_hash_str);
        
        ROS_INFO("model hash => %zu", model_hash);
        ROS_INFO("hash string => %s", model_hash_str.c_str());
        num_classes = pdetectNet->GetNumClasses();

        for( uint32_t n=0; n < num_classes; n++ )
		{
            class_descriptions.push_back(pdetectNet->GetClassDesc(n));
        }

        // // create the key on the param server
        class_key = std::string("class_labels_") + std::to_string(model_hash);

		pDetectetNetNodePrivate->setParam(class_key, class_descriptions);
        // // ROS_DECLARE_PARAMETER(class_key, class_descriptions);
        // // ROS_SET_PARAMETER(class_key, class_descriptions)

        // // ROS_CREATE_PUBLISHER(vision_msgs::Detection2DArray, "detections", 25, pDetectionPublisher);
	    // ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);
	    // ROS_CREATE_PUBLISHER_STATUS(vision_msgs::VisionInfo, "vision_info", 1, info_callback, info_pub);

    }
}

void DetectnetWrapper::Detect(sensor_msgs::ImagePtr input)
{
	detection2DArray.detections.clear();

    // convert the image to reside on GPU
	if( !input_cvt || !input_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

    // classify the image
	detectNet::Detection* detections = NULL;

	const int numDetections = pdetectNet->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), 
												&detections, detectNet::OVERLAY_NONE);

	// verify success	
	if( numDetections < 0 )
	{
		ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
		return;
	}

	// if objects were detected, send out message
	if( numDetections > 0 )
	{
		// ROS_INFO("detected %i objects in %ux%u image", numDetections, input->width, input->height);
		
		// create a detection for each bounding box

		for( int n=0; n < numDetections; n++ )
		{
			detectNet::Detection* det = detections + n;

			// ROS_INFO("object %i class #%u (%s)  confidence=%f", n, det->ClassID, pdetectNet->GetClassDesc(det->ClassID), det->Confidence);
			// ROS_INFO("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height()); 
			
			// create a detection sub-message
			vision_msgs::Detection2D detMsg;

			detMsg.bbox.size_x = det->Width();
			detMsg.bbox.size_y = det->Height();
			
			float cx, cy;
			det->Center(&cx, &cy);

			detMsg.bbox.center.x = cx;
			detMsg.bbox.center.y = cy;

			detMsg.bbox.center.theta = 0.0f;		// TODO optionally output object image

			// create classification hypothesis
			vision_msgs::ObjectHypothesisWithPose hyp;
			
			hyp.id = det->ClassID;
			hyp.score = det->Confidence;
		
			detMsg.results.push_back(hyp);
			detection2DArray.detections.push_back(detMsg);
		}
	}

	PublishOverlay(detections, numDetections);

}

void DetectnetWrapper::PublishDetections()
{
		// populate timestamp in header field
		detection2DArray.header.stamp = ROS_TIME_NOW();

		// publish the detection message
		detectionPublisher.publish(detection2DArray);
}

bool DetectnetWrapper::PublishOverlay( detectNet::Detection* detections, int numDetections)
{
	// get the image dimensions
	const uint32_t width  = input_cvt->GetWidth();
	const uint32_t height = input_cvt->GetHeight();

	// assure correct image size
	if( !overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat) )
		return false;

	// generate the overlay
	if( !pdetectNet->Overlay(input_cvt->ImageGPU(), overlay_cvt->ImageGPU(), width, height, 
				   imageConverter::InternalFormat, detections, numDetections, overlay_flags) )
	{
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;

	if( !overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat) )
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message	
	overlayPublisher.publish(msg);
	ROS_DEBUG("publishing %ux%u overlay image", width, height);
}