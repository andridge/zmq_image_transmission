#include <ros/ros.h>
#include <zmq.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define PFLN printf("LINE %d\n", __LINE__);

class BBox
{
public:
   int x, y,h,w;
   std::string category;
   float c;

    BBox(int x, int y, int h, int w, std::string category, float c)
    {
       this->x = x;
       this->y = y;
       this->h = h;
       this->w = w;
       this->category = category;
       this->c = c;
    }

    void drawOnImage(cv::Mat* image)
    {

       cv::line(*image,cv::Point(this->x,this->y),cv::Point(this->x + this->w,this->y),cv::Scalar(255,0,0),2);
       cv::line(*image,cv::Point(this->x,this->y+this->h),cv::Point(this->x + this->w,this->y+this->h),cv::Scalar(255,0,0),2);

       cv::line(*image,cv::Point(this->x,this->y),cv::Point(this->x,this->y+this->h),cv::Scalar(255,0,0),2);
       cv::line(*image,cv::Point(this->x+this->w,this->y),cv::Point(this->x+this->w,this->y+this->h),cv::Scalar(255,0,0),2);

    }

};

int main(int argc, char** argv)
{
    zmq::context_t context(1);
    zmq::socket_t  get_image_from_ROS(context, ZMQ_REQ);

    std::printf("Connecting to server a... \n");
    get_image_from_ROS.connect ("tcp://127.0.0.1:6666");

    zmq::context_t context_go(2);
    zmq::socket_t  give_image_to_ROS(context_go, ZMQ_REP);
    give_image_to_ROS.bind("tcp://*:5555");

    while (1) {
        // Request new image
        zmq::message_t request(5);
        memcpy(request.data(), "Hello", 5);
        std::printf("Sending request... \n");
        get_image_from_ROS.send(request);

        // get the reply
        zmq::message_t reply;
        get_image_from_ROS.recv(&reply);
        std::vector <uchar> buffer;
        std::printf("Received reply: %d bytes %d \n", static_cast<int>(reply.size()), 480 * 640 * 3);
        // store the reply data into an image structure
        cv::Mat ROS_image(480, 640, CV_8UC3, reply.data());

        // Create random bbox
        BBox bbox1 = BBox(100, 50, 40, 70, "tomate", 90);
        BBox bbox2 = BBox(300, 250, 70, 40, std::string("pera"), 40);

        // Send BBoxes to bbox receiver
        std::string str_bboxes;
        str_bboxes += "100,50,40,70,tomate,90;";


        PFLN
        size_t bboxes_msg_size = str_bboxes.length();
        PFLN
        zmq::message_t request_bboxes(bboxes_msg_size);
        PFLN
        memcpy(request_bboxes.data(), str_bboxes.c_str(), bboxes_msg_size);
        PFLN
        give_image_to_ROS.send(request_bboxes);
        PFLN
        zmq::message_t reply_bboxes;
        PFLN
        give_image_to_ROS.recv(&reply_bboxes);
        PFLN
//

        bbox1.drawOnImage(&ROS_image);
        bbox2.drawOnImage(&ROS_image);

        cv::imshow("zmq_image_receiver", ROS_image);
        cv::waitKey(5000);



    }
}
