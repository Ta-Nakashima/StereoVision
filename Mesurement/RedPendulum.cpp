/****
振り子までの距離を計測するプログラム
note. 実験環境は背景をブルーシートで多い，検出対象は赤色の振り子(1つ)のみとする.
****/

#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdlib.h>
#include <cmath>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 100
#define B_MIN 0
#define G_MAX 100
#define G_MIN 0
#define R_MAX 255
#define R_MIN 100
//ステレオカメラの情報
#define Focus    338
#define Interval 100
#define CamWidth 320
#define CamHight 240

//プロトタイプ宣言
void Combine(Mat *Dual_Cam, Mat ReCmb1, Mat ReCmb2, int dir);
int Detect_HorizontalPosition(Mat im, int d);
double Pixel(int d);
double Angle(int d);

// メイン関数
int main(int argc, char *argv[])
{
  //カメラ1オープン
  VideoCapture cap1(0);  //カメラデバイスの取り込み
  do{
    cap1.open(0);
    waitKey(10);
  }
  while(!cap1.isOpened());
  if(!cap1.isOpened()){
    std::cout << "Errer! Can't open device" << '\n';
    return -1;
  }
  cap1.set(CAP_PROP_FRAME_WIDTH, CamWidth);    //カメラ画像1の横幅を320に設定
  cap1.set(CAP_PROP_FRAME_HEIGHT, CamHight);   //カメラ画像1の縦幅を240に設定

  //カメラ2オープン
  VideoCapture cap2(1);  //カメラデバイスの取り込み
  do{
    cap2.open(1);
    waitKey(10);
  }while(!cap2.isOpened());
  if(!cap2.isOpened()){
    std::cout << "Errer! Can't open device" << '\n';
    return -1;
  }
  cap2.set(CAP_PROP_FRAME_WIDTH, CamWidth);    //カメラ画像2の横幅を320に設定
  cap2.set(CAP_PROP_FRAME_HEIGHT, CamHight);   //カメラ画像2の縦幅を240に設定

  //動画保存
  //VideoWriter writer("./movie/result.mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 15, Size(640, 480), true);
  //if (!writer.isOpened()){ return -1; }

  //textファイルに記録
  ofstream outputfile("record.txt");
  outputfile << "視差" << setw(12) << "|  測定結果" << endl;

  while(1){
    Mat input_image_rgb1;
    Mat input_image_rgb2;

    cap1 >> input_image_rgb1;
    cap2 >> input_image_rgb2;

    //imwrite("test.jpg", input_image_rgb1);

    //左右カメラの映像を連結
    Mat Stereo = Mat(Size(input_image_rgb1.cols * 2, input_image_rgb1.rows), CV_8UC3, Scalar::all(0));
    Combine(&Stereo, input_image_rgb1, input_image_rgb2, 0);

    // 結果保存用Matを定義
    Mat mask_image, output_image_rgb;

    // inRangeを用いてフィルタリング
    Scalar s_min = Scalar(B_MIN, G_MIN, R_MIN);
    Scalar s_max = Scalar(B_MAX, G_MAX, R_MAX);
    inRange(Stereo, s_min, s_max, mask_image);

    // マスクを基に入力画像をフィルタリング
    Stereo.copyTo(output_image_rgb, mask_image);

    //元の映像と赤色抽出映像を上下に連結
    Mat result = Mat(Size(Stereo.cols, Stereo.rows * 2), CV_8UC3, Scalar::all(0));
    Combine(&result, Stereo, output_image_rgb, 1);

    //視差の検出と距離計算
    int d;  //視差(Pixel差)
    double LinerDistance;  // 線形近似
    double PixelDistance;  // 1/4inc変換
    double AngleDistance;  // 角度から変換
    d = Detect_HorizontalPosition(output_image_rgb, d);
    PixelDistance = Pixel(d); // dを1280pixel換算に戻す
    AngleDistance = Angle(d);

    //textファイルに記録
    outputfile << d << " pixel | " << setw(6) << PixelDistance << " mm |" << endl;

    char value_c[256]; //次の行で使う一時的な変数
    sprintf(value_c, "d: %3d pixcel | Z: %4.3f mm", d, PixelDistance); //変数の値も含めた表示したい文字列をchar型変数に格納
    putText(result, value_c, Point(70,230), FONT_HERSHEY_TRIPLEX, 1, Scalar(0,200,200), 1, CV_AA);

    // 結果の表示と保存
    imshow("result", result);
    //writer << result;

    //qボタンでループ終了
    int key = cv::waitKey(1);
    if(key == 113){
      break;
    }
  }
  outputfile.close();
  destroyAllWindows();
  return 0;
}

//左右カメラの結合
void Combine(Mat *result, Mat ReCmb1, Mat ReCmb2, int dir){

  //合成前のイメージを合成後のイメージにコピーする領域を作り,２つを関連付ける.
  if (dir == 0){  //左右の結合
    cv::Mat im1(*result, cv::Rect(0, 0, ReCmb1.cols, ReCmb1.rows));
    cv::Mat im2(*result, cv::Rect(ReCmb1.cols, 0, ReCmb1.cols, ReCmb1.rows));
    //合成前のイメージを合成後のイメージにコピーする。
    ReCmb1.copyTo(im1);
    ReCmb2.copyTo(im2);
  }else if (dir == 1){  //上下の結合
    cv::Mat im1(*result, cv::Rect(0, 0, ReCmb1.cols, ReCmb1.rows));
    cv::Mat im2(*result, cv::Rect(0, ReCmb1.rows, ReCmb1.cols, ReCmb1.rows));
    //合成前のイメージを合成後のイメージにコピーする。
    ReCmb1.copyTo(im1);
    ReCmb2.copyTo(im2);
  }

  return;
}

//振り子の位置測定及び視差の計算
int Detect_HorizontalPosition(Mat im, int d){

  int min1 = 0;
  int min2 = 0;
  int max1, max2;
  double pos[2] = {};

  //左右カメラ内の赤画素の範囲を探索
  for (size_t i = 0; i < CamWidth; i++) {
    for (size_t j = 0; j < CamHight; j++) {
      if(im.at<Vec3b>(j, i)[2] != 0){
        if (min1 == 0) {
          min1 = i;
        }else{
          max1 = i;
        }
      }
      if(im.at<Vec3b>(j, i+CamHight)[2] != 0){
        if (min2 == 0) {
          min2 = i;
        }else{
          max2 = i;
        }
      }
    }
  }

  //赤画素の中心を振り子の重心とする
  pos[0] = 179 - (max1 + min1)/2; // Cx : 179 picel
  pos[1] = 183 - (max2 + min2)/2; // Cx : 183 pixel
  cout << max1 << " , " << min1 << endl;
  //視差dの計算
  d = pos[0] - pos[1];

  return abs(d);
}

//pixel->mm変換による距離計測
double Pixel(int d){
  double distance;

  distance = (Interval * Focus) / d;

  return distance;
}

//角度による距離計測
double Angle(int d){
  double distance;
  double parallax;
  double angle;

  parallax = (3.6 * d) / 1280; // 1/4incセンサからの比率
  angle = atan2(Focus, parallax);
  distance = (tan(angle) * Interval);

  return distance;
}
