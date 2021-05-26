#include <ros/ros.h>
#include <zmq.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat img;
cv::Mat imagerec;
bool has_img=false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

	has_img=false;
  try{
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Size sz1(640,480);
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
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);

    //  Prepare our context and publisher
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_REP);
    publisher.bind("tcp://192.168.30.101:6666");

    ROS_INFO("ZMQ _ SERVER LISTNING \n");

    int count = 0;
    while (ros::ok()){

	if(has_img){
		//std::stringstream msg_str_stream;
		//msg_str_stream << imagerec.data << count;
		//std::string msg_str = msg_str_stream.str();
   		zmq::message_t request;

		if (publisher.recv(&request, ZMQ_DONTWAIT)){
		        size_t frameSize = imagerec.step[0] * imagerec.rows;
			publisher.send((void*) imagerec.data, frameSize);
		}
 		
		cv::imshow("view", imagerec);
     		cv::waitKey(30);
        
}
		loop_rate.sleep();
		count++;
		ros::spinOnce();
    }
    // Clean up your socket and context here
    publisher.close();
    context.close();

    return 0;
}
