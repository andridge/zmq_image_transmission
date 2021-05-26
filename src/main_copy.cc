/*
 * Copyright (c) 2016-2018 DeePhi Tech, Inc.
 *
 * All Rights Reserved. No part of this source code may be reproduced
 * or transmitted in any form or by any means without the prior written
 * permission of DeePhi Tech, Inc.
 *
 * Filename: main.cc
 * Version: 1.10
 *
 * Description:
 * Sample source code showing how to deploy ResNet50 neural network on
 * DeePhi DPU@Zynq7020 platform.
 */
// modified by daniele.bagni@xilinx.com for minVggNet CNN.
// date 20 April 2018

#include <assert.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <atomic>
#include <sys/stat.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <mutex> 
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <dnndk/dnndk.h>
#include <opencv2/opencv.hpp>
#include <zmq.hpp>

using namespace cv;
using namespace std;
using namespace std::chrono;

int threadnum;
//#define RESNET50_WORKLOAD_CONV (7.71f)
//#define RESNET50_WORKLOAD_FC (4.0f / 1000)

#define KERNEL_CONV "miniVggNet_0"
//#define KERNEL_FC "resnet50_2"
#define CONV_INPUT_NODE "conv1"
#define CONV_OUTPUT_NODE "fc2" 
//#define FC_INPUT_NODE  "fc1"  //"fc1000"
//#define FC_OUTPUT_NODE "fc2" //"fc1000"

const string baseImagePath = "labels/";

//#define SHOWTIME

#ifdef SHOWTIME
#define _T(func)                                                              \
    {                                                                         \
        auto _start = system_clock::now();                                    \
        func;                                                                 \
        auto _end = system_clock::now();                                      \
        auto duration = (duration_cast<microseconds>(_end - _start)).count(); \
        string tmp = #func;                                                   \
        tmp = tmp.substr(0, tmp.find('('));                                   \
        cout << "[TimeTest]" << left << setw(30) << tmp;                      \
        cout << left << setw(10) << duration << "us" << endl;                 \
    }
#else
#define _T(func) func;
#endif
/**
 * @brief put image names to a vector
 *
 * @param path - path of the image direcotry
 * @param images - the vector of image name
 *
 * @return none
 */
void ListImages(string const &path, vector<string> &images) {
    images.clear();
    struct dirent *entry;
    /*Check if path is a valid directory path. */
    struct stat s;
    lstat(path.c_str(), &s);
    if (!S_ISDIR(s.st_mode)) {
        fprintf(stderr, "Error: %s is not a valid directory!\n", path.c_str());
        exit(1);
    }
    DIR *dir = opendir(path.c_str());
    if (dir == nullptr) {
        fprintf(stderr, "Error: Open %s path failed.\n", path.c_str());
        exit(1);
    }
    while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_type == DT_REG || entry->d_type == DT_UNKNOWN) {
            string name = entry->d_name;
	    //cout << "DBG ListImages: " << name << endl;
            string ext = name.substr(name.find_last_of(".") + 1);
            if ((ext == "JPEG") || (ext == "jpeg") || (ext == "JPG") || (ext == "jpg") ||
                (ext == "PNG") || (ext == "png")) {
                images.push_back(name);
            }
        }
    }

    closedir(dir);
}

/**
 * @brief load kinds from file to a vector
 *
 * @param path - path of the kind file
 * @param kinds - the vector of kinds string
 *
 * @return none
 */
void LoadWords(string const &path, vector<string> &kinds) {
    kinds.clear();
    fstream fkinds(path);
    if (fkinds.fail()) {
        fprintf(stderr, "Error : Open %s failed.\n", path.c_str());
        exit(1);
    }string kind;
    while (getline(fkinds, kind)) {
        kinds.push_back(kind);
    }
    fkinds.close();
}

/**
 * @brief softmax operation
 *
 * @param data - pointer to input buffer
 * @param size - size of input buffer
 * @param result - calculation result
 *
 * @return none
 */
void CPUCalcSoftmax(const float *data, size_t size, float *result, Mat img_out, cv::Rect roi) {
    assert(data && result);
    double sum = 0.0f;
    for (size_t i = 0; i < size; i++) {
        result[i] = exp(data[i]);
        sum += result[i];
    }
    for (size_t i = 0; i < size; i++) {
        result[i] /= sum;
    }
	float max_prob_id[2];
    int max_id[2];
    float max_prob=0.0;
    for(int ii=0;ii<4;ii++){
	    if(result[ii]>max_prob){
			max_prob=result[ii];
			max_id[0]=ii;
			max_prob_id[0]=result[ii];	
	      }
     }

    if(max_prob_id[0] > 0.90) {
		    if(max_id[0] == 0 ){
				     cv::rectangle(img_out, roi, cv::Scalar(0, 255, 0));
		    }else if(max_id[0] == 1){
				      cv::rectangle(img_out, roi, cv::Scalar(0, 0,255));
		    }else if(max_id[0] == 2){
			    cv::rectangle(img_out, roi, cv::Scalar(255, 0,0));
		    }else if(max_id[0] == 3){
			    cv::rectangle(img_out, roi, cv::Scalar(255, 255,255));
		    }
		    }
}

/**H
 * @brief Get top k results according to its probability
 *
 * @param d - pointer to input data
 * @param size - size of input data
 * @param k - calculation result
 * @param vkinds - vector of kinds
 *
 * @return none
 */
void TopK(const float *d, int size, int k, vector<string> &vkind) {
    assert(d && size > 0 && k > 0);
    priority_queue<pair<float, int>> q;
    for (auto i = 0; i < size; ++i) {
        q.push(pair<float, int>(d[i], i));
    }
    for (auto i = 0; i < k; ++i){
      pair<float, int> ki = q.top();
      printf("[Top]%d prob = %-8f  name = %s\n", i, d[ki.second], vkind[ki.second].c_str());
      q.pop();
    }
}


/**
 * @brief Run CONV Task for miniVggNet
 *
 * @param taskConv - pointer to miniVggNet CONV Task
 *
 * @return none
 */
// daniele.bagni@xilinx.com

vector<string> kinds, images;
void run_miniVggNet(DPUTask *taskConv, Mat img,  Mat img_out, cv::Rect roi) {
    assert(taskConv );
    int channel = dpuGetOutputTensorChannel(taskConv, CONV_OUTPUT_NODE);
    float *softmax = new float[channel];
    float *FCResult = new float[channel];
    _T(dpuSetInputImage2(taskConv, CONV_INPUT_NODE, img));
    _T(dpuRunTask(taskConv));
    _T(dpuGetOutputTensorInHWCFP32(taskConv, CONV_OUTPUT_NODE, FCResult, channel));
    _T(CPUCalcSoftmax(FCResult, channel, softmax, img_out, roi));
   // _T(TopK(softmax, channel, 5, kinds));   
    delete[] softmax;
    delete[] FCResult;
}
    
//void classifyEntry(DPUKernel *kernelconv, DPUKernel *kernelfc) {
void classifyEntry(DPUKernel *kernelconv){
    bool flag=false;
    /* Load all kinds words.*/
    LoadWords(baseImagePath + "labels.txt", kinds);
    if (kinds.size() == 0) {
        cerr << "\nError: Not words exist in words.txt." << endl;
        return;
    }
    thread workers[threadnum];
    auto _start = system_clock::now(); 
    std::cout << "open ZMQ" << std::endl;
    int cameraIndex = 0;
    // initialize the zmq context and socket
    zmq::context_t context(1);
    zmq::context_t context_go(2);
    zmq::socket_t  get_image_from_ROS(context, ZMQ_REQ);
    zmq::socket_t  give_image_to_ROS(context_go, ZMQ_REP);
    give_image_to_ROS.bind("tcp://*:5555");
    // zmq::socket_t subscriber(context, ZMQ_SUB);
    //std::string TOPIC = "";
    //subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length()); // allow all messages
    int linger = 0; // Proper shutdown ZeroMQ
    cv::VideoCapture img(cameraIndex);
    if (flag){
	    // initialize the camera capture
        if(!img.isOpened()){
        std::printf("Unable to open video source, terminating program! \n");
        return;
        }
        int imgWidth = static_cast<int>(img.get(CV_CAP_PROP_FRAME_WIDTH));
        int imgHeight = static_cast<int>(img.get(CV_CAP_PROP_FRAME_HEIGHT));
        std::printf("Video source opened successfully (width=%d height=%d)! \n", imgWidth, imgHeight);
    }else{
	    // subscriber.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));
        std::printf("Connecting to server tcp://192.168.30.101:6666... \n");
        // connect to the image server
	    std::printf("Connecting to server a... \n");
	    get_image_from_ROS.connect ("tcp://192.168.30.101:6666");
    }
	DPUTask *taskconv[threadnum];
    for (auto i = 0; i < threadnum; i++){
    workers[i] = thread([&,i](){
  
#define DPU_MODE_NORMAL 0
#define DPU_MODE_PROF   1
#define DPU_MODE_DUMP   2	        
	// Create DPU Tasks from DPU Kernel
    taskconv[i] = dpuCreateTask(kernelconv, DPU_MODE_NORMAL); // profiling not enabled
    //DPUTask *taskconv = dpuCreateTask(kernelconv, DPU_MODE_PROF); // profiling enabled
	//enable profiling
	//int res1 = dpuEnableTaskProfile(taskconv);
	//if (res1!=0) printf("ERROR IN ENABLING TASK PROFILING FOR CONV KERNEL\n");  
    });
    }
	 bool doimg = true;
	 int frameCount = 0;
	 zmq::message_t request(5);
	 memcpy(request.data(), "Hello", 5);
     int buffer_of_results[20];
     while(doimg){
         auto begin = system_clock::now();                                       
		// attempt to acquire an image frame
	    bool captureSuccess = false;
        cv::Mat captureFrame(480, 640, CV_8UC3);
        cv::Mat processedFrame;
	    if(flag){
	        std::cout << "I am capturing from CAM\n" << std::endl;
	        captureSuccess = img.read(captureFrame);
	     }else{
        // send the request
		    std::printf("Sending request... \n");
		    get_image_from_ROS.send(request);
            // get the reply
            zmq::message_t reply;
            get_image_from_ROS.recv(&reply);
            std::vector<uchar> buffer;
            std::printf("Received reply: %d bytes %d \n", static_cast<int>(reply.size()), 480*640*3);
            // store the reply data into an image structure
            cv::Mat ROS_image(480, 640, CV_8UC3, reply.data());
            captureFrame  = ROS_image;
            ROS_image.copyTo( captureFrame );
            captureSuccess=true;		
            }
        std::cout << "GO\n" << std::endl;
        if(captureSuccess){
            std::cout << "Captured\n" << std::endl;
            // process the image frame
            // create the message container
            Size sz(32,32);
		    cv::Mat crop;
            int dpu_ki_deploy=0; 
            for(int jj=0; jj<20;jj++){
		       for(int ji=0; ji<10;ji++){
                    cv::Rect roi;
                    roi.x = 0+jj*30;
                    roi.y = 0+ji*45;
                    roi.width = 50;
                    roi.height = 50;
                    crop = captureFrame(roi);
                    Mat img2; resize(crop, img2, sz); //DB
                    run_miniVggNet(taskconv[dpu_ki_deploy],img2, captureFrame, roi );
                    dpu_ki_deploy++;
                    if(dpu_ki_deploy>=threadnum){
                        dpu_ki_deploy=0;
                    }
                    //printf("%d %lf\n", max_id[0], max_prob[0] );	
                    //putText(captureFrame, kinds[max_id[0]].c_str(), cvPoint(roi.x,roi.y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		        }
             }

        // poll to see if a message has arrived
            zmq::message_t request_fromROS;
            if (give_image_to_ROS.recv(&request_fromROS, ZMQ_DONTWAIT)){
                    printf("Received request...GET result \n");
                    size_t frameSize = captureFrame.step[0] * captureFrame.rows;
                    give_image_to_ROS.send((void*) captureFrame.data, frameSize);
             }else{
	     	printf("NO request");
	     }
            // increment the frame counter
            frameCount++;
            }else{
                    printf("Unable to acquire image frame! \n");
                 }
         auto _end= system_clock::now();   
         auto duration = (duration_cast<microseconds>(_end- begin ).count() );   
         cout << "[Time]" << duration << "us" << endl;  
	}
    for (auto i = 0; i < threadnum; i++){
    workers[i] = thread([&,i](){

#define DPU_MODE_NORMAL 0
#define DPU_MODE_PROF   1
#define DPU_MODE_DUMP   2	    
	    
        // Create DPU Tasks from DPU Kernel
        DPUTask *taskconv = dpuCreateTask(kernelconv, DPU_MODE_NORMAL); // profiling not enabled
        //DPUTask *taskconv = dpuCreateTask(kernelconv, DPU_MODE_PROF); // profiling enabled
        //enable profiling
        //int res1 = dpuEnableTaskProfile(taskconv);
        //if (res1!=0) printf("ERROR IN ENABLING TASK PROFILING FOR CONV KERNEL\n");
	
for(unsigned int ind = i  ;ind < images.size();ind+=threadnum){
	    Mat img = imread(baseImagePath + images.at(ind));
	    cout << "DBG imread " << baseImagePath + images.at(ind) << endl;
	    //Size sz(32,32);
	    //Mat img2; resize(img, img2, sz); //DB
        //run_miniVggNet(taskconv,img2); //DB: images are already 32x32 and do not need any resize
	    // run_miniVggNet(taskconv,img);
	}
	// Destroy DPU Tasks & free resources
	     dpuDestroyTask(taskconv);
    });
    }
    // Release thread resources.
    for (auto &w : workers) {
        if (w.joinable()) w.join();
    }     
    auto _end = system_clock::now();                                       
    auto duration = (duration_cast<microseconds>(_end - _start)).count();   
    cout << "[Time]" << duration << "us" << endl;  
    cout << "[FPS]" << images.size()*1000000.0/duration  << endl;  
}

/**
 * @brief Entry for running miniVggNet neural network
 *
 */
int main(int argc ,char** argv)
{
    printf("we are compiling this file\n");
    if(argc == 2)
		threadnum = stoi(argv[1]);

    DPUKernel *kernelConv;
    dpuOpen();
    kernelConv = dpuLoadKernel(KERNEL_CONV);
    classifyEntry(kernelConv);
    dpuDestroyKernel(kernelConv);
    dpuClose();
    return 0;
}

