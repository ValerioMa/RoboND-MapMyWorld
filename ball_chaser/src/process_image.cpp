#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <list>

namespace enc = sensor_msgs::image_encodings;

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Moving the arm: (lin_x, ang_z) = (" << lin_x << ", " << ang_z <<");" );

    // Create service msg
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

void process_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    const double max_speed = 0.5;
    const double max_angular = 2.5;


    // Convert to rgb      
    cv_bridge::CvImageConstPtr cv_ptr; 
    cv_ptr = cv_bridge::toCvShare(msg,  enc::BGR8);
    cv::Mat src_gray;
    cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    cv::GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

    /// Apply the Hough Transform to find the circles
    std::vector<cv::Vec3f> circles;
    //cv::threshold(src_gray, src_gray, 100, 255,cv::THRESH_TOZERO); //Threshold the gray
    cv::HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 70, 35, 0, 0 );


  /// Draw the circles detected
  for( size_t i = 0; i < circles.size(); i++ )
  {
      cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      cv::circle( src_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
      // circle outline
      cv::circle( src_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
   }

  /// Show your results
  cv::namedWindow( "Hough Circle Transform Demo", CV_WINDOW_AUTOSIZE );
  cv::imshow( "Hough Circle Transform Demo", src_gray );

  cv::waitKey(10);

  double lin_x = 0;
  double ang_z = 0; //max_angular;
  if(circles.size() != 1){
    ROS_WARN_STREAM(circles.size() << " circles detected");  
  }else{
    // Loop through each pixel in the image and check if its equal to the first one
    ang_z = - (2*(circles[0][0]/(double)src_gray.cols) - 1.0)*max_angular;
    double speed_scale = std::max(std::min(1., (90 - circles[0][2])/10.),0.);
    lin_x = speed_scale*max_speed;
  }

    // If the image is uniform and the arm is not moving, move the arm to the center
    drive_robot(lin_x, ang_z);    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
