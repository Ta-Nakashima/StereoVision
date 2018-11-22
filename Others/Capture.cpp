#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

int main(int argh, char* argv[])
{
  cv::VideoCapture cap(0);//デバイスのオープン
  //cap.open(0);//こっちでも良い．
  cap.set(CAP_PROP_FRAME_WIDTH, 320);    //カメラ画像1の横幅を320に設定
  cap.set(CAP_PROP_FRAME_HEIGHT, 240);   //カメラ画像1の縦幅を240に設定


  if(!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
  {
    //読み込みに失敗したときの処理
    return -1;
  }

  while(1)//無限ループ
  {
    cv::Mat frame;
    cap >> frame; // get a new frame from camera

    //
    //取得したフレーム画像に対して，クレースケール変換や2値化などの処理を書き込む．
    //

    cv::imshow("window", frame);//画像を表示．

    int key = cv::waitKey(1);
    if(key == 113)//qボタンが押されたとき
    {
      break;//whileループから抜ける．
    }
    else if(key == 115)//sが押されたとき
    {
      static int i = 0;
      std::ostringstream oss;
      oss << std::setfill( '0' ) << std::setw( 3 ) << i++;
      //フレーム画像を保存する．
      //imwrite("img.png", frame);
      imwrite("cam1/cam1_" + oss.str() + ".jpg", frame);
    }
  }
  cv::destroyAllWindows();
  return 0;
}
