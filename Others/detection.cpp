/***
シングルカメラでの確認用
***/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

using namespace std;  //stdを省略できる.
using namespace cv;  //cvを省略できる.
using namespace std::chrono;

//マクロ
#define CAM_WIDTH  320  //カメラの横幅
#define CAM_HIGHT  240  //カメラの縦幅
#define FPS        7.5  //保存する動画のFPS
#define THRESHOLD  50   //最初の二値化の閾値(あんま関係ない)
#define HISTSIZE   125  //作成するヒストグラムの大きさ
#define RESIZE     1    //最終的な映像のサイズ変更時の変更率
//#define THRESHOLD2 60   //ピラミッド処理後に行う二値化の閾値

//プロトタイプ宣言
void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat dif1, Mat dif2, Mat *diff, Mat *result, Mat *color, ofstream &outputfile, ofstream &timefile);
void Cleate_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols, ofstream &outputfile, ofstream &timefile);
int Hist_Ave(int range, float *pos, Mat *hist, int n, ofstream &outputfile, ofstream &timefile);
int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *pos, int ave, Mat *color, int *top, ofstream &outputfile, ofstream &timefile);
int Similar(int cnt, int *top, float *pos, int ave, ofstream &timefile, ofstream &outputfile);
void Object_Line(int scan_dir, int dep_dir, Mat *im, int *top, int cnt, ofstream &outputfile, ofstream &timefile);
void Object_Num(int scan_dir, int dep_dir, Mat *im, int cnt, ofstream &timefile);
void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2, ofstream &outputfile, ofstream &timefile);
int Max_Search(int *top, int cnt, ofstream &outputfile, ofstream &timefile);
int Judge_Rest(int scan_dir, float *pos, int ave, ofstream &outputfile, ofstream &timefile);
void Reframe(Mat *ReCmb, int col, int row, Mat hist_rows, Mat hist_cols, Mat color, Mat number, ofstream &outputfile, ofstream &timefile);
void Combine(Mat *Dual_Cam, Mat ReCmb1, Mat ReCmb2, ofstream &outputfile, ofstream &timefile);

int main(int argc, char *argv[])
{
  Mat im1, im2, tmp, tmp2;
  Mat dif1, dif2, dstt, diff1, diff2, result1, result2, color, color2, abst;
  Mat frame_l1, frame_l2, frame_l3, frame_l4, frame_l5, frame_l6, frame_l7;
  Mat frame_r1, frame_r2, frame_r3, frame_r4, frame_r5, frame_r6, frame_r7;
  int col = CAM_WIDTH; //画像の横幅
  int row = CAM_HIGHT; //画像の縦幅
  int cnt = 0;
  int num = 0;
  int ans = 0;
  int i;

  //fpsカウンター用変数
  int frame_cnt = 0;
  int scond = 0;
  int64 nowTime = 0;    // 現時刻
  int64 diffTime = 0;   // 経過時間
  int fps = 0;    // 1秒のフレーム数
  const double f = (1000 /cv::getTickFrequency());

  //カメラ1オープン
  VideoCapture cap1(0);  //カメラデバイスの取り込み
  VideoCapture cap2(1);  //カメラデバイスの取り込み
  do{
    cap1.open(0);
    cap2.open(1);
  }
  while( !cap1.isOpened() || !cap2.isOpened() );
  if( !cap1.isOpened() || !cap2.isOpened() ){
    std::cout << "Errer! Can't open device" << '\n';
    return -1;
  }
  cap1.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);    //カメラ画像1の横幅を320に設定
  cap1.set(CAP_PROP_FRAME_HEIGHT, CAM_HIGHT);   //カメラ画像1の縦幅を240に設定
  cap2.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);    //カメラ画像2の横幅を320に設定
  cap2.set(CAP_PROP_FRAME_HEIGHT, CAM_HIGHT);   //カメラ画像2の縦幅を240に設定

  std::cout << endl << "---------[操作方法]---------" << endl;
  std::cout << "[Space]でデータの記録を開始します．" << endl;
  std::cout << "[Enter]で記録を終了します．" << endl;
  std::cout << "プログラムを終了する際は[Esc]を押してください." << endl;
  std::cout << "---------------------------" << endl << endl;

  string mp;

  cout << "実行結果を動画に保存しますか？" << endl;
  cout << "(1)Yes (2)No" << endl << "=>";
  while(true){
    try{
      cin >> ans;
    }
    catch(...)
    {
      std::cout << "指定された数字で入力してください．" << '\n';
      std::cout << "(1)Yes (2)No" << '\n' << "=>";
      cin.clear();//例外を消去。
      cin.seekg(0);//よくわかりませんがとりあえず書く。
      continue;
    }
    break;
  }

  if(ans == 1){
    #define MOV  //動画を保存する場合のマクロ定義
    std::cout << endl << "記録するファイル名を指定してください(拡張子除く)．" << '\n' << "RecordName : ";
    cin >> mp;
  }

  #ifdef MOV

  VideoWriter result("record/" + mp + "_result.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size((CAM_WIDTH + HISTSIZE) * 2,CAM_HIGHT + HISTSIZE), true);
  if(!result.isOpened()){
    cout << "ERR : result_MOV can't open" << endl;
    return -1;
  }
  VideoWriter  pyr_l("record/" + mp + "_pyr_l.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size(CAM_WIDTH, CAM_HIGHT), false);
  if(!pyr_l.isOpened()){
    cout << "ERR : pyr_l_MOV can't open" << endl;
    return -1;
  }
  VideoWriter  pyr_r("record/" + mp + "_pyr_r.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size(CAM_WIDTH, CAM_HIGHT), false);
  if(!pyr_r.isOpened()){
    cout << "ERR : pyr_r_MOV can't open" << endl;
    return -1;
  }
  VideoWriter  binary_l("record/" + mp + "_bin_l.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size(CAM_WIDTH, CAM_HIGHT), false);
  if(!binary_l.isOpened()){
    cout << "ERR : bin_l_MOV can't open" << endl;
    return -1;
  }
  VideoWriter binary_r("record/" + mp + "_bin_r.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size(CAM_WIDTH, CAM_HIGHT), false);
  if(!binary_r.isOpened()){
    cout << "ERR : bin_r_MOV can't open" << endl;
    return -1;
  }
  VideoWriter origin_l("record/" + mp + "_origin_l.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size(CAM_WIDTH, CAM_HIGHT), true);
  if(!origin_l.isOpened()){
    cout << "ERR : ori_l_MOV can't open" << endl;
    return -1;
  }
  VideoWriter origin_r("record/" + mp + "_origin_r.mov", VideoWriter::fourcc('m','p','4','v'), FPS, cv::Size(CAM_WIDTH, CAM_HIGHT), true);
  if(!origin_r.isOpened()){
    cout << "ERR : ori_r_MOV can't open" << endl;
    return -1;
  }
  ofstream outputfile("record/" + mp + "_data.txt");
  ofstream timefile("record/" + mp + "_time.txt");
  #endif

  cout << endl << "差分を取るフレームの枚数を選んでください．" << endl;
  cout << "(1)3フレーム (2)5フレーム (3)7フレーム" << endl << "=>";
  cin >> num;
  while(num < 1 || num > 3){
    std::cout << "指定された数字で入力してください．" << '\n';
    std::cout << "(1)3フレーム (2)5フレーム (3)7フレーム" << '\n' << "=> ";
    cin >> num;
  }

  //fps計測用タイマー
  int64 startTime = cv::getTickCount();

  /* 以下動画像処理 */
  while (1){

    cap1 >> im1;
    cap2 >> im2;
    //imshow("im1", im1);
    //imshow("im2", im2);
    cvtColor(im1, frame_l1, CV_BGR2GRAY);  //グレースケール化
    cvtColor(im2, frame_r1, CV_BGR2GRAY);  //グレースケール化

    int key = cv::waitKey(1);
    //Spaceキーで記録開始
    if(key == 32){
      #define TXT  //処理時間の記録開始
      //Enterキーで記録終了
      if(key == 10){
        #ifdef REC
        outputfile.close();
        #endif
        //#undef TXT
        break;
      }
    }

    //何秒，何フレーム目かをメモ
    std::cout << scond << "s, " << frame_cnt << "f" << endl;
    #ifdef TXT
    outputfile << "---------------------------------------------------" << endl;
    outputfile << scond << "s, " << frame_cnt << "f" << endl;
    timefile << "-----------------------------------------------------" << endl;
    timefile << scond << "s, " << frame_cnt << "f" << endl;
    #endif

    //目的のフレーム数が集まり次第差分計算を開始
    if((num == 1 && cnt == 2) || (num == 2 && cnt == 4) || (num == 3 && cnt == 6)){

      //フレーム数ごとの差分計算
      if(num == 1){
        Diff_Cal(frame_l1, frame_l2, frame_l3, dif1, dif2, &diff1, &result1, &color, outputfile, timefile);
        Diff_Cal(frame_r1, frame_r2, frame_r3, dif1, dif2, &diff2, &result2, &color2, outputfile, timefile);
      }else if(num == 2){
        Diff_Cal(frame_l1, frame_l3, frame_l5, dif1, dif2, &diff1, &result1, &color, outputfile, timefile);
        Diff_Cal(frame_r1, frame_r3, frame_r5, dif1, dif2, &diff2, &result2, &color2, outputfile, timefile);
      }else if(num == 3) {
        Diff_Cal(frame_l1, frame_l4, frame_l7, dif1, dif2, &diff1, &result1, &color, outputfile, timefile);
        Diff_Cal(frame_r1, frame_r4, frame_r7, dif1, dif2, &diff2, &result2, &color2, outputfile, timefile);
      }

      //ヒストグラムの作成
      Mat histl_rows = Mat(Size(col, HISTSIZE), CV_8UC3, Scalar::all(0));  //rows*500のウィンドウ配列
      Mat histl_cols = Mat(Size(HISTSIZE, row), CV_8UC3, Scalar::all(0));  //HISTSIZE*colsのウィンドウ配列
      Mat histr_rows = Mat(Size(col, HISTSIZE), CV_8UC3, Scalar::all(0));  //rows*500のウィンドウ配列
      Mat histr_cols = Mat(Size(HISTSIZE, row), CV_8UC3, Scalar::all(0));  //HISTSIZE*colsのウィンドウ配列
      float histl_x[col];  //正規化した縦軸方向の白画素数格納用配列(cam1)
      float histl_y[row];  //正規化した横軸方向の白画素数格納用配列(cam1)
      float histr_x[col];  //正規化した縦軸方向の白画素数格納用配列(cam2)
      float histr_y[row];  //正規化した横軸方向の白画素数格納用配列(cam2)
      for(i = 0; i < col; i++){
        histl_x[i] = 0;
        histr_x[i] = 0;
      }
      for(i = 0; i < row; i++){
        histl_y[i] = 0;
        histr_y[i] = 0;
      }
      Cleate_Hist(result1, col, row, histl_x, histl_y, &histl_rows, &histl_cols, outputfile, timefile);
      Cleate_Hist(result2, col, row, histr_x, histr_y, &histr_rows, &histr_cols, outputfile, timefile);


      //ヒストグラム平均の描写
      int ave_l1, ave_l2;
      int ave_r1, ave_r2;
      ave_l1 = Hist_Ave(col, histl_x, &histl_rows, 0, outputfile, timefile);
      ave_r1 = Hist_Ave(col, histr_x, &histr_rows, 0, outputfile, timefile);
      ave_l2 = Hist_Ave(row, histl_y, &histl_cols, 1, outputfile, timefile);
      ave_r2 = Hist_Ave(row, histr_y, &histr_cols, 1, outputfile, timefile);
      #ifdef TXT
      outputfile << "cam1_ave_G = " << ave_l1 << endl;
      outputfile << "cam1_ave_B = " << ave_r1 << endl;
      outputfile << "cam2_ave_G = " << ave_l2 << endl;
      outputfile << "cam2_ave_B = " << ave_r2 << endl;
      #endif

      //ヒストグラムの山の数を計測．
      int cnt_l1, cnt_l2;
      int cnt_r1, cnt_r2;
      int top_l1[128] = {};
      int top_r1[128] = {};
      int top_l2[128] = {};
      int top_r2[128] = {};
      Mat number1 = Mat(Size(HISTSIZE, HISTSIZE), CV_8UC3, Scalar::all(150));
      Mat number2 = Mat(Size(HISTSIZE, HISTSIZE), CV_8UC3, Scalar::all(150));

      #ifdef TXT
      outputfile << "------cam1.Green------" << endl;
      #endif
      cnt_l1 = Count_Mass(&number1, col, row, histl_x, ave_l1, &color, top_l1, outputfile, timefile);  //縦ヒストグラム

      #ifdef TXT
      outputfile << "------cam1.Blue------" << endl;
      #endif
      cnt_l2 = Count_Mass(&number1, row, col, histl_y, ave_l2, &color, top_l2, outputfile, timefile);  //横ヒストグラム

      #ifdef TXT
      outputfile << "------cam2.Green------" << endl;
      #endif
      cnt_r1 = Count_Mass(&number2, col, row, histr_x, ave_r1, &color2, top_r1, outputfile, timefile);  //縦ヒストグラム

      #ifdef TXT
      outputfile << "------cam2.Blue------" << endl;
      #endif
      cnt_r2 = Count_Mass(&number2, row, col, histr_y, ave_r2, &color2, top_r2, outputfile, timefile);  //横ヒストグラム

      #ifdef TXT
      char f_time1[128];
      char f_time2[128];
      sprintf(f_time1, "cam1:%2ds, %df", scond, frame_cnt);
      sprintf(f_time2, "cam2:%2ds, %df", scond, frame_cnt);
      putText(number1, f_time1, Point(8,120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0,0,0), 1, CV_AA); //(20,100)の位置に大きさ1、太さ1の黒文字で描画
      putText(number2, f_time2, Point(8,120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0,0,0), 1, CV_AA); //(20,100)の位置に大きさ1、太さ1のm黒文字で描画
      #endif

      //物体の位置を表示
      #ifdef TXT
      outputfile << "--cam1_Circle--" << endl;
      #endif
      Circle_Draw(&color, top_l1, top_l2, cnt_l1, cnt_l2, outputfile, timefile);
      #ifdef TXT
      outputfile << "--cam2_Circle--" << endl;
      #endif
      Circle_Draw(&color2, top_r1, top_r2, cnt_r1, cnt_r2, outputfile, timefile);

      //フレームの連結およびリサイズ
      cv::Mat ReCmb1, ReCmb2;
      Reframe(&ReCmb1, col, row, histl_rows, histl_cols, color, number1, outputfile, timefile);
      Reframe(&ReCmb2, col, row, histr_rows, histr_cols, color2, number2, outputfile, timefile);

      Mat Dual_Cam = Mat(Size(ReCmb1.cols * 2, ReCmb1.rows), CV_8UC3, Scalar::all(0));  //合成用Mat
      Combine(&Dual_Cam, ReCmb1, ReCmb2, outputfile, timefile);

      //fpsの計算
      nowTime = cv::getTickCount();
      diffTime = (int)((nowTime - startTime)*f);  //計測時間
      if (diffTime >= 1000) {   //計測時間が1秒を超えたら実行
        startTime = nowTime;
        fps = frame_cnt;
        frame_cnt = 0;
        scond++;
      }

      #ifdef TXT
      timefile << "fps = " << fps << endl;
      #endif

      //映像の出力
      //imshow("cam1", im1);
      //imshow("cam2", im2);
      imshow("Dual_Cam", Dual_Cam);
      //cvMoveWindow("Dual_Cam", 0, 0);

      //映像の保存
      #ifdef TXT
      result << Dual_Cam;
      binary_l << diff1;
      binary_r << diff2;
      pyr_l << result1;
      pyr_r << result2;
      origin_l << im1;
      origin_r << im2;
      #endif
    }

    //フレームの初期化と交換
    steady_clock::time_point st = steady_clock::now();
    if(cnt == 0){
      frame_l2 = 0;
      frame_r2 = 0;
      frame_l3 = 0;
      frame_r3 = 0;
      frame_l2 = frame_l1.clone();
      frame_r2 = frame_r1.clone();
    }else if(cnt == 1 || (cnt ==2 && num == 1) ){
      frame_l3 = 0;
      frame_r3 = 0;
      frame_l3 = frame_l2.clone();
      frame_r3 = frame_r2.clone();
      frame_l2 = 0;
      frame_r2 = 0;
      frame_l2 = frame_l1.clone();
      frame_r2 = frame_r1.clone();
    }else if(cnt == 2){
      frame_l4 = 0;
      frame_r4 = 0;
      frame_l4 = frame_l3.clone();
      frame_r4 = frame_r3.clone();
      frame_l3 = 0;
      frame_r3 = 0;
      frame_l3 = frame_l2.clone();
      frame_r3 = frame_r2.clone();
      frame_l2 = 0;
      frame_r2 = 0;
      frame_l2 = frame_l1.clone();
      frame_r2 = frame_r1.clone();
    }else if(cnt == 3 || (num == 2 && cnt == 4)){
      frame_l5 = 0;
      frame_r5 = 0;
      frame_l5 = frame_l4.clone();
      frame_r5 = frame_r4.clone();
      frame_l4 = 0;
      frame_r4 = 0;
      frame_l4 = frame_l3.clone();
      frame_r4 = frame_r3.clone();
      frame_l3 = 0;
      frame_r3 = 0;
      frame_l3 = frame_l2.clone();
      frame_r3 = frame_r2.clone();
      frame_l2 = 0;
      frame_r2 = 0;
      frame_l2 = frame_l1.clone();
      frame_r2 = frame_r1.clone();
    }else if(cnt == 4){
      frame_l6 = 0;
      frame_l6 = frame_l5.clone();
      frame_l5 = 0;
      frame_l5 = frame_l4.clone();
      frame_l4 = 0;
      frame_l4 = frame_l3.clone();
      frame_l3 = 0;
      frame_l3 = frame_l2.clone();
      frame_l2 = 0;
      frame_l2 = frame_l1.clone();
      frame_l6 = 0;
      frame_r6 = frame_r5.clone();
      frame_r5 = 0;
      frame_r5 = frame_r4.clone();
      frame_r4 = 0;
      frame_r4 = frame_r3.clone();
      frame_r3 = 0;
      frame_r3 = frame_r2.clone();
      frame_r2 = 0;
      frame_r2 = frame_r1.clone();
    }else if(cnt == 5 || (num == 3 && cnt ==6)){
      frame_l7 = 0;
      frame_l7 = frame_l6.clone();
      frame_l6 = 0;
      frame_l6 = frame_l5.clone();
      frame_l5 = 0;
      frame_l5 = frame_l4.clone();
      frame_l4 = 0;
      frame_l4 = frame_l3.clone();
      frame_l3 = 0;
      frame_l3 = frame_l2.clone();
      frame_l2 = 0;
      frame_l2 = frame_l1.clone();
      frame_r7 = 0;
      frame_r7 = frame_r6.clone();
      frame_r6 = 0;
      frame_r6 = frame_r5.clone();
      frame_r5 = 0;
      frame_r5 = frame_r4.clone();
      frame_r4 = 0;
      frame_r4 = frame_r3.clone();
      frame_r3 = 0;
      frame_r3 = frame_r2.clone();
      frame_r2 = 0;
      frame_r2 = frame_r1.clone();
    }else{
      cout << "!! Errer !!" << endl;
      return -1;
      break;
    }
    steady_clock::time_point en = steady_clock::now();
    steady_clock::duration ab = en - st;
    #ifdef TXT
    timefile << "Change_Flame = " << duration_cast<milliseconds>(ab).count() << " ms\n";
    #endif

    //回数を2まで計測
    steady_clock::time_point s = steady_clock::now();
    if(num == 1){
      if(cnt < 2){
        cnt++;
      }
    }else if(num == 2){
      if(cnt < 4){
        cnt++;
      }
    }else if(num == 3){
      if(cnt < 6){
        cnt++;
      }
    }else{
      cout << "!! Errer !!" << endl;
      return -1;
      break;
    }
    steady_clock::time_point e = steady_clock::now();
    steady_clock::duration abc = e - s;
    #ifdef TXT
    timefile << "Cnt = " << duration_cast<milliseconds>(abc).count() << " ms\n" << endl;
    #endif

    std::cout << '\n';

    //Escキーで終了
    if(key == 27){
      break;
    }

    frame_cnt++;
  }

  destroyAllWindows();// ウィンドウを閉じる．

  return 0;
}

//フレーム間差分計算関数
void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat dif1, Mat dif2,Mat *diff, Mat *result, Mat *color, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();

  Mat abst , abst2, abst3;

  absdiff(frame1, frame2, dif1);  //差分計算1
  absdiff(frame2, frame3, dif2);  //差分計算2
  bitwise_and(dif1, dif2, *diff);  //取得した差分を論理積処理
  threshold(*diff, *result, THRESHOLD, 255, THRESH_BINARY | THRESH_OTSU);    //閾値THRESHOLDで二値化
/*
  pyrDown( *diff, abst, Size( color->cols/2, color->rows/2 ));
  pyrDown( abst, abst2, Size( abst.cols/2, abst.rows/2 ));
  pyrUp( abst2, abst, Size( abst2.cols*2, abst2.rows*2 ));
  pyrUp( abst, *result, Size( abst.cols*2, abst.rows*2 ));
  threshold(*result, *result, THRESHOLD2, 255, THRESH_BINARY);
*/
  cvtColor(*result,*color,CV_GRAY2BGR);  //チャンネル数を3に変更

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Diff_Cal = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}

//ヒストグラム作成関数(参考 >> http://ishidate.my.coocan.jp/opencv_5/opencv_5.htm)
void Cleate_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();
  int i, j;

  //縦方向における白画素の検出
  Mat abst;
  abst = result.t();

  for(i = 0; i < col; i++){               //x座標を移動
    for(j = 0; j < row; j++){             //y座標を移動
      if(abst.at<unsigned char>(i,j) == 255){
        histf_x[i]++;                               //縦軸上の白画素(255)の数をカウント(→)
      }
    }
  }
  //横方向における白画素の検出
  for(j = 0; j < row; j++){               //y座標を移動
    for(i = 0; i < col; i++){             //x座標を移動
      if(result.at<unsigned char>(j,i) == 255){
        histf_y[j]++;                               //横軸上の白画素(255)の数をカウント(↑)
      }
    }
  }

  //最大値の測定
  int max_x = 0;
  int max_y = 0;
  //縦軸上の白画素数の最大値を計測
  for(i = 0; i < col; i++){
    if(histf_x[i] > max_x)
    max_x = histf_x[i];                             //最大値の更新(↑)
  }
  //横軸上の白画素数の最大値を計測
  for(i = 0; i < row; i++){
    if(histf_y[i] > max_y)
    max_y = histf_y[i];                             //最大値の更新(→)
  }

  #ifdef TXT
  outputfile << "ヒストグラム最大値(緑) = " << max_x << endl;
  outputfile << "ヒストグラム最大値(青) = " << max_y << endl;
  #endif

  //最大値で正規化
  //縦軸方向の正規化とラインの描画
  for(i = 0; i < col; i++){
    histf_x[i] = (((float)histf_x[i] * HISTSIZE) / (float)max_x);  //500で正規化(→)
    //40pix毎に赤線を引く
    if(i % 40 == 0){
      line(*hist_rows, Point(i, 0), Point(i, HISTSIZE), Scalar(0,0,255), 1, 4);
    }else{
      line(*hist_rows, Point(i, HISTSIZE), Point(i, HISTSIZE - (int)histf_x[i]), Scalar(255,255,255), 1, 4);
    }
  }
  //横軸方向の正規化とラインの描画
  for(i = 0; i < row; i++){
    histf_y[i] = (((float)histf_y[i] * HISTSIZE) / (float)max_y);  //500で正規化(↑)
    //40pix毎に赤線を引く
    if(i % 40 == 0){
      line(*hist_cols, Point(0, i), Point(HISTSIZE, i), Scalar(0,0,255), 1, 4);
    }else{
      line(*hist_cols, Point(0, i), Point((int)histf_y[i], i), Scalar(255,255,255), 1, 4);
    }
  }
  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Cleate_Hist = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}

//ヒストグラムの平均を計算し画像上に表示する関数
int Hist_Ave(int range, float *pos, Mat *hist, int n, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();

  int i;
  int ave = 0;
  float sum = 0;

  //縦軸上ヒストグラムの平均を計算
  for(i = 0, sum = 0; i < range; i++){
    sum += pos[i];
  }
  ave = (int)((int)(sum) / range);
  if(n == 0){
    line(*hist, Point(0, HISTSIZE - ave), Point(range, HISTSIZE - ave), Scalar(0,255,0), 1, 4);
  }else if(n == 1){
    line(*hist, Point(ave, 0), Point(ave, range), Scalar(255,0,0), 1, 4);
  }

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Hist_Ave = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return ave;
ı}

//ヒストグラム上の山の数を計測
int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *pos, int ave, Mat *color,  int *top, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();

  #define REPETE  3      //連続性が途絶えた場合の許容回数
  #define RANGE   10      //連続性の幅
  #define ST_HI   0      //開始,終了点の高さ

  int i, j, k;
  double slope;                  //傾き
  int tmp, judge;                //条件カウンター
  int cnt = 0;                   //条件カウンター
  int check = 0;                 //条件カウンター,2までカウントされれば物体有りと判定
  int st = 0, tp = 0, en = 0;    //山の始点,頂点,終点

  //動物体があるか判断
  judge = Judge_Rest(scan_dir, pos, ave, outputfile, timefile);
  if(judge == 0){
    cnt = 0;
    check = 0;
  }else if(judge == 1){
    //ヒストグラムを走査
    for(i = 0; i < scan_dir; i++){
      //平均値以上の値が検出された場合，ループ文に入る
      if((int)pos[i] > ave + ST_HI){
        st = i; //開始地点
        check = 0;
        //山の登りが連続か判定,allow_num回まで偽判定を許容(条件1)
        for(tmp = 0; tmp < REPETE; tmp++){
          //現地点からRANGE分隣のpixとの角度計算
          slope = atan2((double)(pos[i + RANGE] - pos[i]), (double)RANGE);
          //傾きが0.5以上で登りと判断
          while(slope > 0.5){

            tmp = 0;
            check = 1;
            i += RANGE;

            //範囲を超えたら強制終了
            if(i > scan_dir){
              check = 2;
              tp = scan_dir;
              en = scan_dir;
              #ifdef TXT
              outputfile << "SKIP1" << '\n';
              #endif
              goto SKIP;  //SKIPラベルまで飛ぶ
            }
            //傾きの更新
            slope = atan2((double)(pos[i + RANGE] - pos[i]), (double)RANGE);
            #ifdef TXT
            outputfile << "SLOPE_UP = " << slope << endl;
            #endif
          }
          //while文を通ってなければiを更新
          if(tmp != 0){
            i += RANGE;
            //範囲を超えれば強制終了
            if(i > scan_dir){
              if(check == 1){
                check = 2;
                tp = i - RANGE;
                en = scan_dir;
                #ifdef TXT
                outputfile << "SKIP2" << '\n';
                #endif
                goto SKIP;
              }else if(check == 0){
                #ifdef TXT
                outputfile << "SKIP3" << '\n';
                #endif
                goto SKIP;
              }
            }
          }
        }
        i -= RANGE * (tmp - 1);  //判定のためいきすぎた分戻す
        tp = en = i;  //山の頂点,終点(暫定値)


        //条件1を満たした場合下りの連続性判定を行う(条件2)
        if(check == 1){
          //条件2
          for(tmp = 0; tmp < REPETE; tmp++){
            slope = atan2((double)(pos[i + RANGE] - pos[i]), (double)RANGE);
            while(slope < - 0.5){

              tmp = 0;
              check = 2;
              i += RANGE;

              //範囲を超えれば強制終了
              if(i > scan_dir){
                en = scan_dir;
                #ifdef TXT
                outputfile << "SKIP4" << '\n';
                #endif
                goto SKIP;
              }
              //ヒストグラムが平均以下の場合強制的に終点とする
              if(pos[i] < ave + ST_HI){
                en = i;
                goto SKIP;
              }

              slope = atan2((double)(pos[i + RANGE] - pos[i]), (double)RANGE);
              #ifdef TXT
              outputfile << "SLOPE_DOWN = " << slope << endl;
              #endif
            }

            if(tmp != 0){
              i += RANGE;
              //範囲を超えれば強制終了
              if(i > scan_dir){
                if(check == 2){
                  en = scan_dir;
                  #ifdef TXT
                  outputfile << "SKIP5" << '\n';
                  #endif
                  goto SKIP;
                }else if(check == 1){
                  goto SKIP;
                }
              }
            }
          }
          i -= RANGE * (tmp - 1);  //判定のため行きすぎた分戻す
          en = i;  //終了点
        }

        SKIP:  //gotoラベル

        #ifdef TXT
        outputfile << "st = " << st << ", top = " << tp << ", en = " << en << endl;
        #endif

        //条件1,2を満たしていれば山の数をカウントし，描写
        if(check == 2){
          if(judge == 1){
            slope = atan2((double)(pos[tp] - pos[st]), (double)(tp - st));
            #ifdef TXT
            outputfile << "C_SLOPE_UP = " << slope << endl;
            #endif
            slope = atan2((double)(pos[en] - pos[tp]), (double)(en - tp));
            #ifdef TXT
            outputfile << "C_SLOPE_DOWN = " << slope << endl << "**" << endl;
            #endif

            top[cnt] = tp;
            cnt++;
          }
        }
      }
    }
  }

  //物体が二つ以上の時,隣り合った頂点が同一物体かを判定
  if(cnt > 1){
    cnt = Similar(cnt, top, pos, ave, timefile, outputfile);
  }

  //物体があればその位置に線を描画
  if(cnt > 0){
    Mat im1 = *color;
    Object_Line(scan_dir, dep_dir, &im1, top, cnt, timefile, outputfile);
  }

  //フレームに個数を描写
  Mat im2 = *number;
  Object_Num( scan_dir, dep_dir, &im2, cnt, timefile);

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Count_Mass = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif


  return cnt;
}

//検出した物体が同一物体かどうかを判定
int Similar(int cnt, int *top, float *pos, int ave, ofstream &timefile, ofstream &outputfile){

  int i, j, k;
  int range;
  int cnt_n = 0;

  steady_clock::time_point start = steady_clock::now();

  outputfile << endl << "cnt = " << cnt << endl;

  for(i = 0; i < cnt-1; i++){
    range = top[i+1] - top[i];
    outputfile << "range = " << range << endl;
    for(j = top[i]; j < top[i] + range; j++){
      //二つの頂点間にave以下の値があれば別物体と判定
      outputfile << "位置,高さ = " << j << "," << pos[j] << endl;
      if(pos[j] == 0){
        cnt_n++;
      }
    }

    outputfile << "obj,cnt_n = " << i << "," << cnt_n << endl;

    //0がrangeの2割以上ならば別物体とする
    if(cnt_n < (int)(range / 10 * 2)){
      outputfile << "top = " << top[i] << "," << top[i+1] << endl;
      //値の大きい頂点を残す
      if(pos[top[i]] >= pos[top[i+1]]){
        for(k = i + 1; k < cnt; k++){
          top[k] = top[k+1];
        }
      }else{
        for(k = i + 1; k < cnt; k++){
          top[k-1] = top[k];
        }
      }
      i--;
      cnt--;
    }
    cnt_n = 0;
  }
  outputfile << "end_cnt = " << cnt << endl << endl;

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Similar = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return cnt;
}

//Count_Massで検出された位置を表示
void Object_Line(int scan_dir, int dep_dir, Mat *im, int *top, int cnt, ofstream &timefile, ofstream &outputfile){

  int i;

  steady_clock::time_point start = steady_clock::now();

  for (i = 0; i < cnt; i++) {
    if(scan_dir > dep_dir){
      //検出位置に縦方向の緑線を描画
      cv::line(*im, Point(top[i], 0), Point(top[i], dep_dir), Scalar(0,255,0), 1, 1);
      #ifdef TXT
      outputfile << "検出位置(緑) = " << top[i] << endl;
      #endif
    }else if(scan_dir < dep_dir){
      //検出位置に横方向の青線を描画
      cv::line(*im, Point(0, top[i]), Point(dep_dir, top[i]), Scalar(255,0,100), 1, 1);
      #ifdef TXT
      outputfile << "検出位置(青) = " << top[i] << endl;
      #endif
    }
  }

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Object_Line = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}

//Count_Massで検出された物体数の表示
void Object_Num(int scan_dir, int dep_dir, Mat *im, int cnt, ofstream &timefile){

  steady_clock::time_point start = steady_clock::now();

  if(scan_dir > dep_dir){
    char width_num[128];
    sprintf(width_num, "%d", cnt);
    putText(*im, "Object", Point(15,73), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,255,50), 1, CV_AA); //(10,85)の位置に大きさ1、太さ1の緑文字で描画
    putText(*im, width_num, Point(50,98), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(0,255,50), 1, CV_AA); //(50,120)の位置に大きさ1、太さ1の緑文字で描画
  }else if(scan_dir < dep_dir){
    char hight_num[128];
    sprintf(hight_num, "%d", cnt);
    putText(*im, "Object", Point(15,30), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(255,0,0), 1, CV_AA); //(10,25)の位置に大きさ1、太さ1の青色文字で描画
    putText(*im, hight_num, Point(50,53), FONT_HERSHEY_TRIPLEX, 0.75, Scalar(255,0,0), 1, CV_AA); //(50,60)の位置に大きさ1、太さ1の青色文字で描画
  }

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Object_Num = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}

//円の描画
void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2, ofstream &outputfile, ofstream &timefile){

  #define WIDTH  10  //判定範囲の横幅
  #define HIGHT  10  //判定範囲の縦幅
  #define HARF   5   //WIDTH/2とかにするとエラーでる...
  #define PER    3   //判定範囲内に存在する物体の割合

  int i, j;
  int x, y;
  int pix = 0;

  steady_clock::time_point start = steady_clock::now();

  Mat imSub = color->clone();

  for (i = 0; i < cnt1; i++) {
    for (j = 0; j < cnt2; j++) {

      Mat imroi;

      //与えられた交点から半径10の矩型を作成(端のエラー処理)
      if(top1[i] < (WIDTH / 2) && top2[j] < (HIGHT / 2)){  //左上
        imroi = imSub(Rect(0, 0, (WIDTH / 2) + top1[i], (HIGHT / 2) + top2[j]));
      }else if(top1[i] < (WIDTH / 2) && top2[j] > imSub.rows - (HIGHT / 2)){  //左下
        imroi = imSub(Rect(0, top2[j] - (HIGHT / 2), (WIDTH / 2) + top1[i], (HIGHT / 2) + (imSub.rows - top2[j])));
      }else if(top1[i] > imSub.cols - (WIDTH / 2) && top2[j] < (HIGHT / 2)){  //右上
        imroi = imSub(Rect(top1[i] - (WIDTH / 2), 0, (WIDTH / 2) + (imSub.cols - top1[i]), (HIGHT / 2) + top2[j]));
      }else if(top1[i] > imSub.cols - (WIDTH / 2) && top2[j] > imSub.rows - (HIGHT / 2)){  //右下
        imroi = imSub(Rect(top1[i] - (WIDTH / 2), top2[j] - (HIGHT / 2), (WIDTH / 2) + (imSub.cols - top1[i]), (HIGHT / 2) + (imSub.rows - top2[j])));
      }else if(top1[i] < (WIDTH / 2)){ //左端
        imroi = imSub(Rect(0, top2[j] - (HIGHT / 2), (WIDTH / 2) + top1[i], HIGHT));
      }else if(top1[i] > imSub.cols - (WIDTH / 2)){  //右端
        imroi = imSub(Rect(top1[i] - HARF, top2[j] - HARF, (WIDTH / 2) + (imSub.cols - top1[i]), HIGHT));
      }else if(top2[j] < (HIGHT / 2)){  //上端
        imroi = imSub(Rect(top1[i] - (WIDTH / 2), 0, WIDTH, (HIGHT / 2) + top2[j]));
      }else if(top2[j] > imSub.rows - (HIGHT / 2)){  //下端
        imroi = imSub(Rect(top1[i] - (WIDTH / 2), top2[j] - (HIGHT / 2), WIDTH, (HIGHT / 2) + (imSub.rows - top2[j])));
      }else{
        imroi = imSub(Rect(top1[i] - (WIDTH / 2), top2[j] - (HIGHT / 2), WIDTH, HIGHT));
      }

      //矩型内の白画素(255)の数をカウント
      for (y = 0; y < imroi.cols; y++) {
        for (x = 0; x < imroi.rows; x++) {
          if(imroi.at<unsigned char>(x,y) > 128){
            pix++;
          }
        }
      }

      #ifdef TXT
      outputfile << "(i,j) = " << i << " , " << j << endl;
      outputfile << "top[i], top[j] = " << top1[i] << top2[j] << endl;
      outputfile << "pix = " << pix << '\n';
      outputfile << "area = " << ((imroi.rows * imroi.cols) / 10) * PER <<'\n';
      #endif

      //矩型内の白画素が指定量を超えてたら物体があると判定
      if(pix > ( (int)((imroi.rows * imroi.cols) / 10) * PER) ){
        cv::circle(*color, cv::Point(top1[i], top2[j]), 10, cv::Scalar(0,0,200), 2, 1);
      }
      pix = 0;
    }
  }

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Circle_Draw = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}

//静止状態かの判別関数1
int Judge_Rest(int scan_dir, float *pos, int ave, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();

  //goll代数 >> 判定の関数
  int i;
  int max = 0;
  int num = 0;

  for(i = 0; i < scan_dir; i++){
    //最大値を測定
    if(pos[i] > max){
      max = pos[i];
    }
    //白画素数が平均の半分以上のものをカウント
    if(pos[i] > (int)(ave / 10) * 5){
      num++;
    }
  }
  //全体の7割以上が平均の半分より大きければ物体無しと判断し0を返す
  //また，最大値が30以下でも物体なしとする
  //
  if(ave > 50 || num > (int)(scan_dir / 10) * 7 || max < 30){
    steady_clock::time_point end = steady_clock::now();
    steady_clock::duration d = end - start;
    #ifdef TXT
    timefile << "Judge_Rest = " << duration_cast<milliseconds>(d).count() << " ms\n";
    #endif

    return 0;
  }else{
    steady_clock::time_point end = steady_clock::now();
    steady_clock::duration d = end - start;
    #ifdef TXT
    timefile << "Judge_Rest = " << duration_cast<milliseconds>(d).count() << " ms\n";
    #endif

    return 1;
  }

}

//フレームの連結およびリサイズ(参考 >> http://dronevisionml.blogspot.jp/2015/08/opencv_20.html)
void Reframe(Mat *ReCmb, int col, int row, Mat hist_rows, Mat hist_cols, Mat color, Mat number, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();

  int cmb_width = HISTSIZE + col;    //合成後横幅
  int cmb_hight = HISTSIZE + row;    //合成後縦幅
  Mat Hist_FrameDiff = Mat(Size(HISTSIZE + col, HISTSIZE + row), CV_8UC3, Scalar::all(0));  //合成用Mat

  //合成前のイメージを合成後のイメージにコピーする領域を作り,２つを関連付ける.
  cv::Mat cmb_im(Hist_FrameDiff, cv::Rect(HISTSIZE, 0, col, row));
  cv::Mat cmb_histX(Hist_FrameDiff, cv::Rect(0, 0, HISTSIZE, row));
  cv::Mat cmb_histY(Hist_FrameDiff, cv::Rect(HISTSIZE, row , col, HISTSIZE));
  cv::Mat cmb_num(Hist_FrameDiff, cv::Rect(0, row , HISTSIZE, HISTSIZE));

  //合成前のイメージを合成後のイメージにコピーする。
  color.copyTo(cmb_im);
  hist_cols.copyTo(cmb_histX);
  hist_rows.copyTo(cmb_histY);
  number.copyTo(cmb_num);

  cv::resize(Hist_FrameDiff, *ReCmb, cv::Size(), RESIZE, RESIZE);
  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Reframe = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}

//左右カメラの結合
void Combine(Mat *Dual_Cam, Mat ReCmb1, Mat ReCmb2, ofstream &outputfile, ofstream &timefile){
  steady_clock::time_point start = steady_clock::now();

  //合成前のイメージを合成後のイメージにコピーする領域を作り,２つを関連付ける.
  cv::Mat right(*Dual_Cam, cv::Rect(ReCmb1.cols, 0, ReCmb1.cols, ReCmb1.rows));
  cv::Mat left(*Dual_Cam, cv::Rect(0, 0, ReCmb1.cols, ReCmb1.rows));

  //合成前のイメージを合成後のイメージにコピーする。
  ReCmb1.copyTo(right);
  ReCmb2.copyTo(left);

  steady_clock::time_point end = steady_clock::now();
  steady_clock::duration d = end - start;
  #ifdef TXT
  timefile << "Combine = " << duration_cast<milliseconds>(d).count() << " ms\n";
  #endif

  return;
}
