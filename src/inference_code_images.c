#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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
using namespace std;
using namespace cv;
int main(int argc, char** argv)
{
    const string classes[1] = { "tomato" };
    auto yolo = vitis::ai::YOLOv3::create("yolov4_tomato");
    Mat img;
    img = imread(argv[1], IMREAD_COLOR);
    auto t1 = std::chrono::high_resolution_clock::now();
    auto results = yolo->run(img);
    for (auto& box : results.bboxes) {
        int label = box.label;
        float xmin = box.x * img.cols;
        float ymin = box.y * img.rows;
        float xmax = xmin + box.width * img.cols;
        float ymax = ymin + box.height * img.rows;
        if (xmin < 0.)
            xmin = 1.;
        if (ymin < 0.)
            ymin = 1.;
        if (xmax > img.cols)
            xmax = img.cols;
        if (ymax > img.rows)
            ymax = img.rows;
        float confidence = box.score;
        if (label > -1) {
            if (label == 0) {
                rectangle(img, Point(xmin, ymin), Point(xmax, ymax),
                    Scalar(200, 0, 0), 1, 1, 0);
                putText(img, classes[label], Point(xmin, ymin - 10),
                    cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(200, 0, 0), 2);
            }
            else {
                rectangle(img, Point(xmin, ymin), Point(xmax, ymax),
                    Scalar(0, 200, 0), 1, 1, 0);
                putText(img, classes[label], Point(xmin, ymin - 10),
                    cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 200, 0), 2);
            }
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    if (img.empty())
        return 1;
    String name = argv[1];
    name += "_detected.jpg";
    imwrite(name, img);
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    std::chrono::duration<double> diff =t2-t1;
    cout << 1000000 / duration << " fps" << endl;
    cout << diff.count()  << " seconds" << endl;
    return 0;
}
