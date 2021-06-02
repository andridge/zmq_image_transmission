//
// Created by mike on 5/26/21.
//

#ifndef ZMQ_IMAGE_TRANSMISSION_BBOXES_H
#define ZMQ_IMAGE_TRANSMISSION_BBOXES_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

vector<string> split(const string& s, char separator)
{
    vector<string> output;

    string::size_type prev_pos = 0, pos = 0;

    while((pos = s.find(separator, pos)) != string::npos)
    {
        string substring( s.substr(prev_pos, pos-prev_pos) );

        output.push_back(substring);

        prev_pos = ++pos;
    }

    output.push_back(s.substr(prev_pos, pos-prev_pos)); // Last word

    return output;
}

class BBox
{
public:
    int x, y,h,w;
    std::string category;
    float c;

    BBox(string str) // overloaded constructor
    {
        vector<string> strs = split(str, ' ');
        this->x = stoi(strs[2]);
        this->y =stoi(strs[3]);
        this->w =stoi(strs[4]);
        this->h =stoi(strs[5]);
        this->category = strs[0];
        this->c = stof(strs[1]);
    }

    BBox(int x, int y, int h, int w, std::string category, float c)
    {
        this->x = x;
        this->y = y;
        this->h = h;
        this->w = w;
        this->category = category;
        this->c = c;
    }

    string getBBoxAsString()
    {
        string str;
        str += this->category;
        str += " ";
        str += to_string(this->c);
        str += " ";
        str += to_string(this->x);
        str += " ";
        str += to_string(this->y);
        str += " ";
        str += to_string(this->w);
        str += " ";
        str += to_string(this->h);

        return str;
    }

    void print(void)
    {
        cout << this->category << ", " << this->c << " x=" << this->x << " y=" << this->y << " w=" << this->w << " h=" << this->h << endl;
    }

    void drawOnImage(cv::Mat* image)
    {

        cv::line(*image,cv::Point(this->x,this->y),cv::Point(this->x + this->w,this->y),cv::Scalar(255,0,0),2);
        cv::line(*image,cv::Point(this->x,this->y+this->h),cv::Point(this->x + this->w,this->y+this->h),cv::Scalar(255,0,0),2);

        cv::line(*image,cv::Point(this->x,this->y),cv::Point(this->x,this->y+this->h),cv::Scalar(255,0,0),2);
        cv::line(*image,cv::Point(this->x+this->w,this->y),cv::Point(this->x+this->w,this->y+this->h),cv::Scalar(255,0,0),2);

    }

};


#endif //ZMQ_IMAGE_TRANSMISSION_BBOXES_H
