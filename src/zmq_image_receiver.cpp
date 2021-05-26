#include <ros/ros.h>
#include <zmq.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "bboxes.h"

#define PFLN printf("LINE %d\n", __LINE__);

using namespace std;


int main(int argc, char** argv)
{
    zmq::context_t context(1);
    zmq::socket_t  zmq_socket(context, ZMQ_REP);

    std::printf("Connecting to server a... \n");
//    zmq_socket.connect ("tcp://127.0.0.1:6666");
    zmq_socket.bind("tcp://127.0.0.1:6666");

    while (1) {


        // Request new image
        zmq::message_t request;
        if (zmq_socket.recv(&request, ZMQ_DONTWAIT)){
            std::vector <uchar> buffer;
//            std::printf("Received request for image classification: %d bytes %d \n", static_cast<int>(request.size()), 480 * 640 * 3);
            cv::Mat ROS_image(480, 640, CV_8UC3, request.data()); // store the reply data into an image structure

            cv::imshow("zmq_image_receiver", ROS_image);

            // Create reply
            vector<BBox> bboxes;
            bboxes.push_back(BBox(100, 50, 40, 70, "tomate", 90)); // Create random bbox
            bboxes.push_back(BBox(300, 250, 70, 40, std::string("pera"), 40));
            bboxes.push_back(BBox(30, 25, 7, 40, std::string("maca"), 40));

            // iterate all bboxes in the bboxes vector
            // Send BBoxes to bbox receiver
            std::string str_bboxes;
            for (size_t i=0; i<bboxes.size(); i++) {
                if (i>0) str_bboxes += ";";
                str_bboxes += bboxes[i].getBBoxAsString();
            }
            cout << str_bboxes << endl;

            zmq::message_t reply;
            zmq_socket.send(zmq::buffer(str_bboxes), zmq::send_flags::dontwait);

        }



        // get the reply
//        zmq::message_t reply;
//        zmq_socket.recv(&reply);
//        std::vector <uchar> buffer;
//        std::printf("Received reply: %d bytes %d \n", static_cast<int>(reply.size()), 480 * 640 * 3);
//        // store the reply data into an image structure
//        cv::Mat ROS_image(480, 640, CV_8UC3, reply.data());
//
//        // Create random bbox
//        BBox bbox1 = BBox(100, 50, 40, 70, "tomate", 90);
//        BBox bbox2 = BBox(300, 250, 70, 40, std::string("pera"), 40);

        // Send BBoxes to bbox receiver
//        std::string str_bboxes;
//        str_bboxes += "100,50,40,70,tomate,90;";




//        bbox1.drawOnImage(&ROS_image);
//        bbox2.drawOnImage(&ROS_image);

        cv::waitKey(500);

    }
}
