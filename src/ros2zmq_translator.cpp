#include <ros/ros.h>
#include <zmq.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <iostream>
#include <vector>

#include "bboxes.h"

using namespace std;
cv::Mat img;
cv::Mat imagerec;
bool has_img=false;


void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    ROS_INFO("Received image message");
    has_img=false;
    try{
        img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Size sz1(640,480); // TODO only works for 640x480 images ...
        cv::resize( img, imagerec, sz1);
        has_img=true;

    }catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker_zmq", ros::init_options::AnonymousName);

    ROS_INFO("running our version \n");

    //ros::Time::init(); // Workaround since we are not using NodeHandle
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/image", 1, imageCallback);

    //  Prepare our context and zmq_socket
    zmq::context_t context(1);
    zmq::socket_t zmq_socket(context, ZMQ_REQ);
    zmq_socket.connect ("tcp://127.0.0.1:6666");

    int count = 0;
    while (ros::ok()){

        if(has_img){

            // Send a new image for classification
            printf("Sending request... \n");
            size_t frameSize = imagerec.step[0] * imagerec.rows;
            zmq_socket.send((void*) imagerec.data, frameSize);
            has_img = false;

            // Receive image classification reply
            printf("Waiting for image classification reply\n");
            zmq::message_t reply;
            zmq_socket.recv(&reply);
            printf("Received reply: %d bytes \n", static_cast<int>(reply.size()));
            string str_reply = string(static_cast<char*>(reply.data()), reply.size());
            cout << "Received reply from image classification: " << str_reply << endl;

            // Parse reply to get BBoxes
            vector<string> str_bboxes = split(str_reply, ';');

            vector<BBox> bboxes;
            for (size_t i=0; i<str_bboxes.size(); i++)
            {
                BBox bbox(str_bboxes[i]);
                bboxes.push_back(bbox);
                cout << to_string(i) << ": " << str_bboxes[i] << endl;
            }

            // iterate all bboxes in the bboxes vector
            for (size_t i=0; i<bboxes.size(); i++) {
                bboxes[i].print();
            }

            cv::imshow("ros2zmq_translator", imagerec);
            cv::waitKey(30);

        }

        loop_rate.sleep();
        count++;
        ros::spinOnce();
    }
    // Clean up your socket and context here
    zmq_socket.close();
    context.close();

    return 0;
}
