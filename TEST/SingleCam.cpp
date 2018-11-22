/****
2台のカメラで同時に物体を検出するプログラム．
****/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>  //勾配計算で使用
#include <chrono>  //fps計算で使用
#include "detection.hpp" //独自関数

using namespace std;
using namespace cv;
using namespace std::chrono;

//マクロ
#define CAM_WIDTH  320  //カメラの横幅
#define CAM_HIGHT  240  //カメラの縦幅
#define HISTSIZE   125  //作成するヒストグラムの大きさ
#define RESIZE     1    //最終的な映像のサイズ変更時の変更率

int main(int argc, char *argv[])
{
  //Class
  Detection dtc;

  Mat im;
  Mat frame1, frame2, frame3;
  Mat result, color;
  Mat hist_rows = Mat(Size(CAM_WIDTH, HISTSIZE), CV_8UC3, Scalar::all(0));  //rows*500のウィンドウ配列
  Mat hist_cols = Mat(Size(HISTSIZE, CAM_HIGHT), CV_8UC3, Scalar::all(0));  //HISTSIZE*colsのウィンドウ配列
  Mat number = Mat(Size(HISTSIZE, HISTSIZE), CV_8UC3, Scalar::all(150));  //検出物体の個数やfpsを表示する領域
  int col = CAM_WIDTH; //画像の横幅
  int row = CAM_HIGHT; //画像の縦幅
  float hist_x[col];  //正規化した縦軸方向の白画素数格納用配列(cam1)
  float hist_y[row];  //正規化した横軸方向の白画素数格納用配列(cam1)
  for(size_t i = 0; i < col; i++){
    hist_x[i] = 0;
  }
  for(size_t i = 0; i < row; i++){
    hist_y[i] = 0;
  }
  int ave1, ave2;  //ヒストグラムの平均を格納
  int cnt = 0;
  int num = 0;
  int ans = 0;
  int cnt1, cnt2;  //Count_Mass関数内で使用する
  int top1[128] = {};  //同上
  int top2[128] = {};  //同上

  //fpsカウンター用変数
  int frame_cnt = 0;
  int scond = 0;
  int64 nowTime = 0;    // 現時刻
  int64 diffTime = 0;   // 経過時間
  int fps = 0;    // 1秒のフレーム数
  const double f = (1000 /cv::getTickFrequency());

  //カメラオープン
  VideoCapture cap(0);  //カメラデバイスの取り込み
  do{
    cap.open(0);
    waitKey(10);
  }
  while(!cap.isOpened());
  if(!cap.isOpened()){
    std::cout << "Errer! Can't open device" << '\n';
    return -1;
  }

  cap.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);    //カメラ画像1の横幅を320に設定
  cap.set(CAP_PROP_FRAME_HEIGHT, CAM_HIGHT);   //カメラ画像1の縦幅を240に設定


  //fps計測用タイマー
  int64 startTime = cv::getTickCount();

  /* 以下動画像処理 */
  while (1){

    //取得した画像をMatへ格納し，グレースケール化
    cap >> im;
    cvtColor(im, frame1, CV_BGR2GRAY);

    //フレーム数ごとの差分計算
    if(cnt == 2){
      dtc.Diff_Cal(frame1, frame2, frame3, &result, &color);

      //ヒストグラムの作成
      dtc.Cleate_Hist(result, col, row, hist_x, hist_y, &hist_rows, &hist_cols);

      //ヒストグラム平均の描写
      ave1 = dtc.Hist_Ave(col, hist_x, &hist_rows, 0);
      ave2 = dtc.Hist_Ave(row, hist_y, &hist_cols, 1);

      //ヒストグラムの山の数を計測．
      cnt1 = dtc.Count_Mass(&number, col, row, hist_x, ave1, &color, top1);  //縦ヒストグラム
      cnt2 = dtc.Count_Mass(&number, row, col, hist_y, ave2, &color, top2);  //横ヒストグラム
      char f_time[32];
      sprintf(f_time, "cam:%2ds, %df", scond, frame_cnt);
      putText(number, f_time, Point(8,120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0,0,0), 1, CV_AA); //(20,100)の位置に大きさ1、太さ1の黒文字で描画

      //重心の検出
      dtc.Circle_Draw(&color, top1, top2, cnt1, cnt2);

      //フレームの連結およびリサイズ
      cv::Mat ReCmb;
      dtc.Reframe(&ReCmb, col, row, hist_rows, hist_cols, color, number);

      //fpsの計算
      nowTime = cv::getTickCount();
      diffTime = (int)((nowTime - startTime)*f);  //計測時間
      if (diffTime >= 1000) {   //計測時間が1秒を超えたら実行
        startTime = nowTime;
        fps = frame_cnt;
        frame_cnt = 0;
        scond++;
      }
      std::cout << "fps = " << fps << '\n';

      //映像の出力
      imshow("result", ReCmb);
    }

    //フレームの初期化と交換
    if(cnt == 0){
      frame2 = 0;
      frame3 = 0;
      frame2 = frame1.clone();
    }else if(cnt == 1 || cnt == 2){
      frame3 = 0;
      frame3 = frame2.clone();
      frame2 = 0;
      frame2 = frame1.clone();
    }

    //回数を1まで計測
    if(cnt < 2){
      cnt++;
    }

    //Escキーで終了
    int key = cv::waitKey(1);
    if(key == 27){
      break;
    }

    frame_cnt++;
  }

  destroyAllWindows();// ウィンドウを閉じる．
  return 0;
}
