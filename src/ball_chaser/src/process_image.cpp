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
    //White pixel Averaging variables
    int white_pixel_xtotal=0;
    int white_pixel_count=0;
    int white_pixel_xmean=0;
    


    //Dividing image width into three so alorithim works independent of width.
    int img_width=img.step/3;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    // Loop through each pixel in the image and check if its equal to the first one

    //Adding an averaging function to ensure all white pixels taken into account
    for (int i = 0; i < img.height * img.step; i++) 
    {
        if (img.data[i] == white_pixel) 
        {
            white_pixel_count++;

            white_pixel_xtotal=white_pixel_xtotal+(i%img.step);
            //ROS_INFO("White pixel @ (i):%1.2f ",(float)(i));
            
        }
    }

   
    if(white_pixel_count>0)
    {
        //Calculated x and y averages
        white_pixel_xmean=white_pixel_xtotal/white_pixel_count;
        
        ROS_INFO("White pixel x centroid @ (x):%1.2f,  L/U Limits:%1.2f , %1.2f ", (float)white_pixel_xmean, (float)(img_width),(float)(img_width)*2);

        if(white_pixel_xmean<(img_width))
        {
                    //Less than 1/3 means ball is to the left
                    //ball_left=true;
            ROS_INFO_STREAM("Left");
            drive_robot(0.0,0.1);

        }
        else if(white_pixel_xmean>(img_width)*2)
        {   
                    //Greater than 2/3 means ball is to the right
                    //ball_right=true;
            ROS_INFO_STREAM("Right");
            drive_robot(0.0,-0.1);
        }
        else if ((img_width)<white_pixel_xmean && white_pixel_xmean<(img_width)*2)
        {
                    //Assumed True Otherwise
                    //ball_mid=true;
            ROS_INFO_STREAM("Forward");
            drive_robot(0.1,0.0);
        }
        else 
        {
                    //Assumed True Otherwise

        }


    }
    else
    {
        //ball_mid=true;
        ROS_INFO_STREAM("No White Pixels Detected, searching...");
        drive_robot(0.0,0.1);
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