/****
1台のカメラで同時に物体を検出するプログラム．
****/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>

using namespace std;  //stdを省略できる.
using namespace cv;  //cvを省略できる.
using namespace std::chrono;

//マクロ
#define CAM_WIDTH  320  //カメラの横幅ı∏
#define CAM_HIGHT  240  //カメラの縦幅
#define FPS        7.5  //保存する動画のFPS
#define THRESHOLD  50   //最初の二値化の閾値(あんま関係ない)
#define THRESHOLD2 60   //ピラミッド処理後に行う二値化の閾値
#define HISTSIZE   125  //作成するヒストグラムの大きさ
#define RESIZE     1    //最終的な映像のサイズ変更時の変更率

//プロトタイプ宣言
void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat dif1, Mat dif2, Mat *diff, Mat *result, Mat *color);
void Cleate_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols);
int Hist_Ave(int range, float *pos, Mat *hist, int n);
int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *pos, int ave, Mat *color, int *top);
int Similar(int cnt, int *top, float *pos, int ave);
void Object_Line(int scan_dir, int dep_dir, Mat *im, int *top, int cnt);
void Object_Num(int scan_dir, int dep_dir, Mat *im, int cnt);
void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2);
int Max_Search(int *top, int cnt);
int Judge_Rest(int scan_dir, float *pos, int ave);
void Reframe(Mat *ReCmb, int col, int row, Mat hist_rows, Mat hist_cols, Mat color, Mat number);
void Combine(Mat *Dual_Cam, Mat ReCmb1, Mat ReCmb2);

int main(int argc, char *argv[])
{
  Mat im, tmp, tmp2;
  Mat dif1, dif2, dstt, diff1, diff2, result1, result2, color, color2, abst;
  Mat frame1, frame2, frame3;
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
  //cap.set(CAP_PROP_FRAME_WIDTH, CAM_WIDTH);    //カメラ画像1の横幅を320に設定
  //cap.set(CAP_PROP_FRAME_HEIGHT, CAM_HIGHT);   //カメラ画像1の縦幅を240に設定


  //fps計測用タイマー
  int64 startTime = cv::getTickCount();

  /* 以下動画像処理 */
  while (1){

    cap >> im;
    int col = im.cols; //画像の横幅
    int row = im.rows; //画像の縦幅
    cvtColor(im, frame1, CV_BGR2GRAY);  //グレースケール化

    int key = cv::waitKey(1);

    //フレーム数ごとの差分計算
    if(cnt == 2){
      Diff_Cal(frame1, frame2, frame3, dif1, dif2, &diff1, &result1, &color);

      //ヒストグラムの作成
      Mat hist_rows = Mat(Size(col, HISTSIZE), CV_8UC3, Scalar::all(0));  //rows*500のウィンドウ配列
      Mat hist_cols = Mat(Size(HISTSIZE, row), CV_8UC3, Scalar::all(0));  //HISTSIZE*colsのウィンドウ配列
      float hist_x[col];  //正規化した縦軸方向の白画素数格納用配列(cam1)
      float hist_y[row];  //正規化した横軸方向の白画素数格納用配列(cam1)
      for(i = 0; i < col; i++){
        hist_x[i] = 0;
      }
      for(i = 0; i < row; i++){
        hist_y[i] = 0;
      }
      Cleate_Hist(result1, col, row, hist_x, hist_y, &hist_rows, &hist_cols);

      //ヒストグラム平均の描写
      int ave1, ave2;
      ave1 = Hist_Ave(col, hist_x, &hist_rows, 0);
      ave2 = Hist_Ave(row, hist_y, &hist_cols, 1);

      //ヒストグラムの山の数を計測．
      int cnt1, cnt2;
      int top1[128] = {};
      int top2[128] = {};
      Mat number = Mat(Size(HISTSIZE, HISTSIZE), CV_8UC3, Scalar::all(150));

      cnt1 = Count_Mass(&number, col, row, hist_x, ave1, &color, top1);  //縦ヒストグラム
      cnt2 = Count_Mass(&number, row, col, hist_y, ave2, &color, top2);  //横ヒストグラム

      char f_time[128];
      sprintf(f_time, "cam:%2ds, %df", scond, frame_cnt);
      putText(number, f_time, Point(8,120), FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0,0,0), 1, CV_AA); //(20,100)の位置に大きさ1、太さ1の黒文字で描画

      Circle_Draw(&color, top1, top2, cnt1, cnt2);

      //フレームの連結およびリサイズ
      cv::Mat ReCmb;
      Reframe(&ReCmb, col, row, hist_rows, hist_cols, color, number);

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
      imshow("ReCmb", ReCmb);
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
    if(key == 27){
      break;
    }

    frame_cnt++;
  }

  destroyAllWindows();// ウィンドウを閉じる．

  return 0;
}

//フレーム間差分計算関数
void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat dif1, Mat dif2,Mat *diff, Mat *result, Mat *color){
  Mat abst , abst2, abst3;

  absdiff(frame1, frame2, dif1);  //差分計算1
  absdiff(frame2, frame3, dif2);  //差分計算2
  bitwise_and(dif1, dif2, *diff);  //取得した差分を論理積処理
  threshold(*diff, *result, THRESHOLD, 255, THRESH_BINARY | THRESH_OTSU);    //閾値THRESHOLDで二値化

  /*pyrDown( *diff, abst, Size( color->cols/2, color->rows/2 ));
  pyrDown( abst, abst2, Size( abst.cols/2, abst.rows/2 ));
  pyrUp( abst2, abst, Size( abst2.cols*2, abst2.rows*2 ));
  pyrUp( abst, *result, Size( abst.cols*2, abst.rows*2 ));

  threshold(*result, *result, THRESHOLD2, 255, THRESH_BINARY);*/
  cvtColor(*result,*color,CV_GRAY2BGR);  //チャンネル数を3に変更

  return;
}

//ヒストグラム作成関数(参考 >> http://ishidate.my.coocan.jp/opencv_5/opencv_5.htm)
void Cleate_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols){
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

  return;
}

//ヒストグラムの平均を計算し画像上に表示する関数
int Hist_Ave(int range, float *pos, Mat *hist, int n){

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

  return ave;
}

//ヒストグラム上の山の数を計測
int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *pos, int ave, Mat *color,  int *top){

  #define REPETE  3      //連続性が途絶えた場合の許容回数
  #define RANGE   3      //連続性の幅
  #define ST_HI   0      //開始,終了点の高さ

  int i, j, k;
  double slope;                  //傾き
  int tmp, judge;                //条件カウンター
  int cnt = 0;                   //条件カウンター
  int check = 0;                 //条件カウンター,2までカウントされれば物体有りと判定
  int st = 0, tp = 0, en = 0;    //山の始点,頂点,終点

  //動物体があるか判断
  judge = Judge_Rest(scan_dir, pos, ave);
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
              goto SKIP;  //SKIPラベルまで飛ぶ
            }
            //傾きの更新
            slope = atan2((double)(pos[i + RANGE] - pos[i]), (double)RANGE);
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
                goto SKIP;
              }else if(check == 0){
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
                goto SKIP;
              }
              //ヒストグラムが平均以下の場合強制的に終点とする
              if(pos[i] < ave + ST_HI){
                en = i;
                goto SKIP;
              }

              slope = atan2((double)(pos[i + RANGE] - pos[i]), (double)RANGE);
            }

            if(tmp != 0){
              i += RANGE;
              //範囲を超えれば強制終了
              if(i > scan_dir){
                if(check == 2){
                  en = scan_dir;
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

        //条件1,2を満たしていれば山の数をカウントし，描写
        if(check == 2){
          if(judge == 1){
            slope = atan2((double)(pos[tp] - pos[st]), (double)(tp - st));
            slope = atan2((double)(pos[en] - pos[tp]), (double)(en - tp));
            top[cnt] = tp;
            cnt++;
          }
        }
      }
    }
  }

  //物体が二つ以上の時,隣り合った頂点が同一物体かを判定
  if(cnt > 1){
    cnt = Similar(cnt, top, pos, ave);
  }

  //物体があればその位置に線を描画
  if(cnt > 0){
    Mat im1 = *color;
    Object_Line(scan_dir, dep_dir, &im1, top, cnt);
  }

  //フレームに個数を描写
  Mat im2 = *number;
  Object_Num( scan_dir, dep_dir, &im2, cnt);

  return cnt;
}

//検出した物体が同一物体かどうかを判定
int Similar(int cnt, int *top, float *pos, int ave){

  int i, j, k;
  int range;
  int cnt_n = 0;

  for(i = 0; i < cnt-1; i++){
    range = top[i+1] - top[i];
    for(j = top[i]; j < top[i] + range; j++){
      //二つの頂点間にave以下の値があれば別物体と判定
      if(pos[j] < ave){
        cnt_n++;
      }
    }

    //0がrangeの2割以上ならば別物体とする
    if(cnt_n < (int)(range / 10 * 2)){
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

  return cnt;
}

//Count_Massで検出された位置を表示
void Object_Line(int scan_dir, int dep_dir, Mat *im, int *top, int cnt){

  int i;

  for (i = 0; i < cnt; i++) {
    if(scan_dir > dep_dir){
      //検出位置に縦方向の緑線を描画
      cv::line(*im, Point(top[i], 0), Point(top[i], dep_dir), Scalar(0,255,0), 1, 1);
    }else if(scan_dir < dep_dir){
      //検出位置に横方向の青線を描画
      cv::line(*im, Point(0, top[i]), Point(dep_dir, top[i]), Scalar(255,0,100), 1, 1);
    }
  }

  return;
}

//Count_Massで検出された物体数の表示
void Object_Num(int scan_dir, int dep_dir, Mat *im, int cnt){

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

  return;
}

//円の描画
void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2){

  #define WIDTH  10  //判定範囲の横幅
  #define HIGHT  10  //判定範囲の縦幅
  #define HARF   5   //WIDTH/2とかにするとエラーでる...
  #define PER    3   //判定範囲内に存在する物体の割合

  int i, j;
  int x, y;
  int pix = 0;

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

      //矩型内の白画素が指定量を超えてたら物体があると判定
      if(pix > ( (int)((imroi.rows * imroi.cols) / 10) * PER) ){
        cv::circle(*color, cv::Point(top1[i], top2[j]), 10, cv::Scalar(0,0,200), 2, 1);
      }
      pix = 0;
    }
  }

  return;
}

//静止状態かの判別関数1
int Judge_Rest(int scan_dir, float *pos, int ave){
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
  if(ave > 50 || num > (int)(scan_dir / 10) * 7 || max < 30){
    return 0;
  }else{
    return 1;
  }
}

//フレームの連結およびリサイズ(参考 >> http://dronevisionml.blogspot.jp/2015/08/opencv_20.html)
void Reframe(Mat *ReCmb, int col, int row, Mat hist_rows, Mat hist_cols, Mat color, Mat number){

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

  return;
}

//左右カメラの結合
void Combine(Mat *Dual_Cam, Mat ReCmb1, Mat ReCmb2){

  //合成前のイメージを合成後のイメージにコピーする領域を作り,２つを関連付ける.
  cv::Mat right(*Dual_Cam, cv::Rect(ReCmb1.cols, 0, ReCmb1.cols, ReCmb1.rows));
  cv::Mat left(*Dual_Cam, cv::Rect(0, 0, ReCmb1.cols, ReCmb1.rows));

  //合成前のイメージを合成後のイメージにコピーする。
  ReCmb1.copyTo(right);
  ReCmb2.copyTo(left);

  return;
}
