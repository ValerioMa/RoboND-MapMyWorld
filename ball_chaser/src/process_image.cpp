#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

#include <list>

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Moving the arm to the center");

    // Create service msg
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service safe_move");
}

void process_image_callback(const sensor_msgs::Image img)
{
    const double max_speed = 0.5;
    const double max_angular = 2.5;

    double lin_x = 0;
    double ang_z = 0; //max_angular;

    int white_pixel = 255;    
    bool ball_finded = false;

    // Loop through each pixel in the image and check if its equal to the first one
    //std::list<std::pair<int,int>> data_list;
    double mean_i = 0, mean_j = 0;
    size_t cnt = 0;    
    for (int i = 0; i < img.height; i++) {
      for(int j=0; (j+3) < img.step; j = j+3){
        int idx = i*img.step + j;
        if(img.data[idx] == white_pixel &&
            img.data[idx + 1] == white_pixel &&
            img.data[idx + 2] == white_pixel){
            mean_i += 2*(i/(double)img.height - 0.5);
            mean_j += 2*(j/(double)img.step - 0.5);
            cnt++;
            //data_list.emplace_back(i,j);
        }
      }        
    }

    if(cnt > 10){
        ball_finded = true;
        mean_i = mean_i/cnt;
        mean_j = mean_j/cnt;
        std::cout << "row position: " << mean_j << std::endl;

        ang_z = - mean_j*max_angular;            
        lin_x = (1 - std::abs(mean_j))*max_speed;
        
        
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
