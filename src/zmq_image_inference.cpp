//#include <ros/ros.h>
#include <zmq.hpp>
#include <zmq.h>
//#include <image_transport/image_transport.h>

//#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "bboxes.h"

#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vitis/ai/demo.hpp>
#include <vitis/ai/yolov3.hpp>
#include <vitis/ai/nnpp/yolov3.hpp>
#include "opencv2/opencv.hpp"
#include <memory>
#include <opencv2/core.hpp>
#include <vector>
#include <vitis/ai/nnpp/yolov3.hpp>
#include <chrono>


#define PFLN printf("LINE %d\n", __LINE__);

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    zmq::context_t context(1);
    zmq::socket_t  zmq_socket(context, ZMQ_REP);

    std::printf("Connecting to server a... \n");
    zmq_socket.connect ("tcp://192.168.200.132:6666");
//    zmq_socket.bind("tcp://127.0.0.1:6666");


    while (1) {

        // Request new image
        zmq::message_t request;
        if (zmq_socket.recv(&request, ZMQ_DONTWAIT)){
            std::vector <uchar> buffer;
//            std::printf("Received request for image classification: %d bytes %d \n", static_cast<int>(request.size()), 480 * 640 * 3);
//            cv::Mat ROS_image(480, 640, CV_8UC3, request.data()); // store the reply data into an image structure
            cv::Mat ROS_image(720, 720, CV_8UC3, request.data()); // store the reply data into an image structure
            // Print the image with label "zmq_image_receiver"
            cv::imshow("zmq_image_inference", ROS_image);

            // Detection using YOLOv4
            const string classes[1] = { "Tomato" };
            auto yolo = vitis::ai::YOLOv3::create("yolov4_tomato");
            //auto t1 = std::chrono::high_resolution_clock::now();
            auto results = yolo->run(ROS_image);


            // Create reply of bounding boxes detection
            vector<BBox> bboxes;
            // Get values of Bounding Boxes
            for (auto& box : results.bboxes) {
                int x = box.x * ROS_image.cols;
                int y = box.y * ROS_image.rows;
                int height = box.height * ROS_image.rows;
                int width = box.width * ROS_image.cols;
                float confidence = box.score;
                int label = box.label;
                bboxes.push_back(BBox(x, y, height, width, "Tomato", confidence));
                if (label > -1) {
                    if (label == 0) {
                        rectangle(ROS_image, Point(x, y), Point(width, height),
                                  Scalar(200, 0, 0), 1, 1, 0);
                        putText(ROS_image, classes[label], Point(x, y - 10),
                                cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(200, 0, 0), 2);
                    }
                    else {
                        rectangle(ROS_image, Point(x, y), Point(width, height),
                                  Scalar(0, 200, 0), 1, 1, 0);
                        putText(ROS_image, classes[label], Point(x, y - 10),
                                cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 200, 0), 2);
                    }
                }

                //bboxes.push_back(BBox(box.x, box.y, box.height, box.width, "Tomato", box.score));
                //int label = box.label;
                //float xmin = box.x * ROS_image.cols;
                //float ymin = box.y * ROS_image.rows;
                //float xmax = xmin + box.width * ROS_image.cols;
                //float ymax = ymin + box.height * ROS_image.rows;
            }
            std::string str_bboxes;

            // Create reply of bounding boxes detection
//            vector<BBox> bboxes;
            // Create random bbox
//            bboxes.push_back(BBox(100, 50, 40, 70, "tomate", 90));
//            bboxes.push_back(BBox(300, 250, 70, 40, std::string("pera"), 40));
//            bboxes.push_back(BBox(30, 25, 7, 40, std::string("maca"), 40));
            // iterate all bboxes in the bboxes vector
            // Send BBoxes to bbox receiver
//            std::string str_bboxes;


            for (size_t i=0; i<bboxes.size(); i++) {
                if (i>0) str_bboxes += ";";
                str_bboxes += bboxes[i].getBBoxAsString();
            }
            //auto t2 = std::chrono::high_resolution_clock::now();
            cout << str_bboxes << endl;

//            mq::message_t message(string.size());
//            std::memcpy (message.data(), string.data(), string.size());
//            bool rc = socket.send (message);

            zmq::message_t reply(str_bboxes.size());
//            zmq_socket.send(zmq::buffer(str_bboxes), zmq::send_flags::dontwait);
            memcpy(reply.data(), str_bboxes.data(), str_bboxes.size());
            zmq_socket.send(reply);
//            zmq::message_t
//            zmq_socket.send()

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

