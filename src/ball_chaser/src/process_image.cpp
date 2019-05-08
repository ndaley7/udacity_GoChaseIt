#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Sending drive command to robot");

    // Request linear and angular values
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service drive_bot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    //Assign Variables
    int white_pixel = 255;
    bool ball_left = false;
    bool ball_mid = false;
    bool ball_right = false;

    //Dividing image width into three so alorithim works independent of width.
    int img_width=img.width/3

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    // Loop through each pixel in the image and check if its equal to the first one
    for(int i=0; i<img.height; i++)
    {
        for(int j=0; j<img.width; j++)
        {
            if (img.data[(i*img.height+j*img.width)] == white_pixel)
            {
                if(j<img_width)
                {
                    //Less than 1/3 means ball is to the left
                    //ball_left=true;
                    ROS_INFO_STREAM("Driving Left");
                    drive_robot(0.1,0.5);

                }
                else if(j>img_width*2)
                {   
                    //Greater than 2/3 means ball is to the right
                    //ball_right=true;
                    ROS_INFO_STREAM("Driving Right");
                    drive_robot(0.1,0.5);
                }
                else
                {
                    //Assumed True Otherwise
                    //ball_mid=true;
                    ROS_INFO_STREAM("Drivng Forward");
                    drive_robot(0.1,0.0);
                }
            
                break;
            }
            }

        }

    }
     
    for (int i = 0; i < img.height * img.step; i++) {
        
    }
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