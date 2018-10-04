#ifndef CAMERA_SENSOR_H
#define CAMERA_SENSOR_H

#include "ros/ros.h"
#include "math.h"
#include "time.h"
#include "sys/time.h"
#include "std_msgs/Float64.h"
#include "opencv2/opencv.hpp"

#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <dirent.h>
#include <iostream>
#include <vector>
#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


static const std::string OPENCV_WINDOW = "Marta Camera Window";



class CameraSensor
{
 private:
  // ROS NodeHandle
  	ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher image_pub_;
		
		int height;
		int width;
		cv::Mat image;
		std::string encoding_;
		


	public:
		bool first;
  	
		CameraSensor(): it_(nh_){
   		image_sub_ = it_.subscribe("/marta_usb_cam/image_raw", 1, &CameraSensor::imageCb, this);
			image_pub_ = it_.advertise("/marta_usb_cam/output_video", 1);	
			cv::namedWindow(OPENCV_WINDOW);
  	}

  	~CameraSensor(){
    	cv::destroyWindow(OPENCV_WINDOW);
  	}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			height = msg->height;
			width = msg->width;


			cv_bridge::CvImagePtr cv_ptr; 
			try 
			{ 
				/* OTHERS ENCODINGS:
						RGB8 
						BGR8
						MONO8
				*/
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
			} 
			catch (cv_bridge::Exception& e) 
			{ 
				ROS_ERROR("cv_bridge exception: %s", e.what()); 
				return; 
			} 


			image = cv_ptr->image;
			//cv::cvtColor(cv_ptr->image, image, CV_RGB2HLS);
			

			cv::waitKey(3); 
			 
			image_pub_.publish(cv_ptr->toImageMsg()); 
  	}

		void show_image()
		{
			if(image.data)
			{
				//namedWIndow("marta_camera_sensor", WINDOW_AUTOSIZE);
				imshow(OPENCV_WINDOW, image);
			}
		}
	
		int get_width()
		{
			return width;
		}

		int get_height()
		{
			return height;
		}

		cv::Mat get_image()
		{
			return image;
		}

		cv::Mat get_image_gray()
		{
			cv::cvtColor(image, image, CV_BGR2GRAY);
			return image;
		}

		cv::Mat get_image_cie()
		{
			cv::cvtColor(image, image, CV_BGR2XYZ);
			return image;
		}
		
		cv::Mat get_image_YCrCb()
		{
			cv::cvtColor(image, image, CV_BGR2YCrCb);
			return image;
		}

		cv::Mat get_image_hsv()
		{
			cv::cvtColor(image, image, CV_BGR2HSV);
			return image;
		}

		cv::Mat get_image_hls()
		{
			cv::cvtColor(image, image, CV_BGR2HLS);
			return image;
		}

		cv::Mat get_image_lab()
		{
			cv::cvtColor(image, image, CV_BGR2Lab);
			return image;
		}

		cv::Mat get_image_luv()
		{
			cv::cvtColor(image, image, CV_BGR2Luv);
			return image;
		}

};


#endif //CAMERA_SENSOR_H

