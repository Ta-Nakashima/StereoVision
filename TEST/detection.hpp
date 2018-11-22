#ifndef _DETECTION_HPP_
#define _DETECTION_HPP_

#include <opencv2/opencv.hpp>

using namespace cv;

// クラス宣言
class Detection
{
public:
    void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat *result, Mat *color);
    void Cleate_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols);
    int Hist_Ave(int range, float *pos, Mat *hist, int n);
    int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *pos, int ave, Mat *color, int *top);
    int Judge_Rest(int scan_dir, float *pos, int ave);
    int Similar(int cnt, int *top, float *pos, int ave);
    void Object_Line(int scan_dir, int dep_dir, Mat *im, int *top, int cnt);
    void Object_Num(int scan_dir, int dep_dir, Mat *im, int cnt);
    void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2);
    void Reframe(Mat *ReCmb, int col, int row, Mat hist_rows, Mat hist_cols, Mat color, Mat number);
private:
    Mat dif1, dif2, con;
    int i, j, k;
    int x, y;
    int max_x, max_y;
};

#endif //_DETECTION_HPP_
