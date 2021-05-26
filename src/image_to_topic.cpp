//
// Created by mike on 5/26/21.
//

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char* argv[])
{
 /*   for (int i = 1; i < argc; i++) {
        std::string s(argv[i]);
        if (s.rfind("--input=", 0) == 0) {
            std::ifstream f(s.substr(std::string("--input=").length()));
            if (f.good()) std::cout << "File exists" << std::endl;
            else std::cout << "File not found" << std::endl;
        } else {
            std::cout << "Unknown: " << s << std::endl;
        }
    }
    exit(0);*/

    cout << argv[1] << endl;
    string image_file = argv[1];

    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    cv_bridge::CvImage cv_image;
    cv_image.image = cv::imread(image_file.c_str(),cv::IMREAD_COLOR);

    cv::imshow("image_to_topic", cv_image.image);
    cv::waitKey(20);

    cv_image.encoding = "bgr8";
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/image", 1);
    ros::Rate loop_rate(5);

    while (nh.ok())
    {
        pub.publish(ros_image);
        loop_rate.sleep();
        cv::waitKey(10);
    }
}