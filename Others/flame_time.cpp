#include "opencv2/opencv.hpp"

using namespace std;  //stdを省略できる.
using namespace cv;  //cvを省略できる.

//マクロ
#define THRESHOLD  50
#define HISTSIZE   500
#define RESIZE     0.8

//プロトタイプ宣言
void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat dif1, Mat dif2, Mat *result, Mat *color);
void Cleat_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols);
int Hist_Ave(int range, float *hist_point, Mat *hist, int n);
int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *hist_point, int ave, Mat *color, int *top);
void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2);
int Max_Search(int *top, int cnt);
int Judge_Rest(int scan_dir, float *hist_point, int ave);
void Reframe(Mat *ReCmb, int col, int row, Mat hist_rows, Mat hist_cols, Mat color, Mat number);

int main(int argc, char *argv[])
{
  Mat im, tmp;
  Mat dif1, dif2, dstt, result, color;
  Mat frame1, frame2, frame3, frame4, frame5, frame6, frame7;

  int cnt = 0;
  int num = 0;
  int ans = 0;
  int i;

  cin.exceptions(ios::failbit);  //cinの例外処理を有効.

  //カメラオープン
  VideoCapture cap(0);  //カメラデバイスの取り込み
  if(!cap.isOpened()){
    std::cout << "Errer! Can't open device" << '\n';
    return -1;
  }
  cap >> tmp;  //画像サイズ取得用

  int row = tmp.rows; //画像の縦幅
  int col = tmp.cols; //画像の横幅

  //動画を保存するかの選択
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
    std::cout << "保存する動画の名前を指定してください(拡張子除く)．" << '\n' << "name : ";
    cin >> mp;
  }

  //動画保存関数
  #ifdef MOV
  VideoWriter writer("/Users/taichi/Dropbox/stereovision/frame/result/" + mp + ".mov", VideoWriter::fourcc('m', 'p', '4', 'v'), 7.5, Size((tmp.cols + HISTSIZE) * RESIZE, (tmp.rows + HISTSIZE) * RESIZE), 1);
  if (!writer.isOpened()){ return -1; }
  #endif

  cout << "差分を取るフレームの枚数を選んでください．" << endl;
  cout << "(1)3フレーム (2)5フレーム (3)7フレーム" << endl << "=>";
  cin >> num;
  while(num < 1 || num > 3){
    std::cout << "指定された数字で入力してください．" << '\n';
    std::cout << "(1)3フレーム (2)5フレーム (3)7フレーム" << '\n' << "=> ";
    cin >> num;
  }

  /* 以下動画像処理 */
  while (1){

    cap >> im;  //カメラからフレームを取得
    cvtColor(im, frame1, CV_BGR2GRAY);  //グレースケール化

    //目的のフレーム数が集まり次第差分計算を開始
    if((num == 1 && cnt == 2) || (num == 2 && cnt == 4) || (num == 3 && cnt == 6)){

      //フレーム数ごとの差分計算
      if(num == 1){
        Diff_Cal(frame1, frame2, frame3, dif1, dif2, &result, &color);
      }else if(num == 2){
        Diff_Cal(frame1, frame3, frame5, dif1, dif2, &result, &color);
      }else if(num == 3) {
        Diff_Cal(frame1, frame4, frame7, dif1, dif2, &result, &color);
      }

      //ヒストグラムの作成
      Mat hist_rows = Mat(Size(col, HISTSIZE), CV_8UC3, Scalar::all(0));  //rows*500のウィンドウ配列
      Mat hist_cols = Mat(Size(HISTSIZE, row), CV_8UC3, Scalar::all(0));  //HISTSIZE*colsのウィンドウ配列
      float histf_x[col];  //正規化した縦軸方向の白画素数格納用配列
      float histf_y[row];  //正規化した横軸方向の白画素数格納用配列
      for(i = 0; i < col; i++){
        histf_x[i] = 0;
      }
      for(i = 0; i < row; i++){
        histf_y[i] = 0;
      }
      Cleat_Hist(result, col, row, histf_x, histf_y, &hist_rows, &hist_cols);

      //ヒストグラム平均の描写
      int ave1, ave2;
      ave1 = Hist_Ave(col, histf_x, &hist_rows, 0);
      ave2 = Hist_Ave(row, histf_y, &hist_cols, 1);

      //ヒストグラムの山の数を計測．
      int cnt1, cnt2;
      int top1[16] = {};
      int top2[16] = {};
      Mat number = Mat(Size(HISTSIZE, HISTSIZE), CV_8UC3, Scalar::all(150));
      cnt1 = Count_Mass(&number, col, row, histf_x, ave1, &color, top1);  //縦ヒストグラム
      cnt2 = Count_Mass(&number, row, col, histf_y, ave2, &color, top2);  //横ヒストグラム

      //物体の位置を表示
      Circle_Draw(&color, top1, top2, cnt1, cnt2);

      //フレームの連結およびリサイズ
      cv::Mat ReCmb;
      Reframe(&ReCmb, col, row, hist_rows, hist_cols, color, number);

      //映像の保存
      #ifdef MOV
      writer << ReCmb;
      #endif

      //映像の出力
      imshow("Hist_FrameDiff", ReCmb);
      cvMoveWindow("Hist_FrameDiff", 0, 0);

      //Escキーで終了
      int key = cv::waitKey(1);
      if(key == 27){
        break;
      }
      //スペースキーで保存
      if(key == 32){
        imwrite("test.jpg", ReCmb);
      }
    }

    //フレームの初期化と交換
    if(cnt == 0){
      frame2 = 0;
      frame3 = 0;
      frame2 = frame1.clone();
    }else if(cnt == 1 || (cnt ==2 && num == 1) ){
      frame3 = 0;
      frame3 = frame2.clone();
      frame2 = 0;
      frame2 = frame1.clone();
    }else if(cnt == 2){
      frame4 = 0;
      frame4 = frame3.clone();
      frame3 = 0;
      frame3 = frame2.clone();
      frame2 = 0;
      frame2 = frame1.clone();
    }else if(cnt == 3 || (num == 2 && cnt == 4)){
      frame5 = 0;
      frame5 = frame4.clone();
      frame4 = 0;
      frame4 = frame3.clone();
      frame3 = 0;
      frame3 = frame2.clone();
      frame2 = 0;
      frame2 = frame1.clone();
    }else if(cnt == 4){
      frame6 = 0;
      frame6 = frame5.clone();
      frame5 = 0;
      frame5 = frame4.clone();
      frame4 = 0;
      frame4 = frame3.clone();
      frame3 = 0;
      frame3 = frame2.clone();
      frame2 = 0;
      frame2 = frame1.clone();
    }else if(cnt == 5 || (num == 3 && cnt ==6)){
      frame7 = 0;
      frame7 = frame6.clone();
      frame6 = 0;
      frame6 = frame5.clone();
      frame5 = 0;
      frame5 = frame4.clone();
      frame4 = 0;
      frame4 = frame3.clone();
      frame3 = 0;
      frame3 = frame2.clone();
      frame2 = 0;
      frame2 = frame1.clone();
    }else{
      cout << "!! Errer !!" << endl;
      return -1;
      break;
    }

    //回数を2まで計測
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
  }

  destroyAllWindows();// ウィンドウを閉じる．

  return 0;
}

//フレーム間差分計算関数
void Diff_Cal(Mat frame1, Mat frame2, Mat frame3, Mat dif1, Mat dif2, Mat *result, Mat *color){

  absdiff(frame1, frame2, dif1);  //差分計算1
  absdiff(frame2, frame3, dif2);  //差分計算2
  bitwise_and(dif1, dif2, *result);  //取得した差分を論理積処理
  threshold(*result, *result, THRESHOLD, 255, THRESH_BINARY | THRESH_OTSU);    //閾値thで二値化
  morphologyEx(*result, *result, MORPH_DILATE, cv::Mat(), cv::Point(-1,-1), 3);  //膨張処理
  morphologyEx(*result, *result, MORPH_ERODE, cv::Mat(), cv::Point(-1,-1), 5);  //縮小処理
  cvtColor(*result,*color,CV_GRAY2BGR);  //チャンネル数を3に変更
}
//ヒストグラム作成関数(参考 >> http://ishidate.my.coocan.jp/opencv_5/opencv_5.htm)
void Cleat_Hist(Mat result, int col, int row, float *histf_x, float *histf_y, Mat *hist_rows, Mat *hist_cols){

  int i, j;

  //縦方向における白画素の検出
  for(i = 0; i < col; i++){               //x座標を移動
    for(j = 0; j < row; j++){             //y座標を移動
      if(result.at<unsigned char>(j,i) == 255){
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
    line(*hist_rows, Point(i, HISTSIZE), Point(i, HISTSIZE - (int)histf_x[i]), Scalar(255,255,255), 1, 4);
  }
  //横軸方向の正規化とラインの描画
  for(i = 0; i < row; i++){
    histf_y[i] = (((float)histf_y[i] * HISTSIZE) / (float)max_y);  //500で正規化(↑)
    line(*hist_cols, Point(0, i), Point((int)histf_y[i], i), Scalar(255,255,255), 1, 4);
  }
}
//ヒストグラムの平均を計算し画像上に表示する関数
int Hist_Ave(int range, float *hist_point, Mat *hist, int n){

  int i;
  int ave = 0;
  float sum = 0;

  //縦軸上ヒストグラムの平均を計算
  for(i = 0, sum = 0; i < range; i++){
    sum += hist_point[i];
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
int Count_Mass(Mat *number, int scan_dir, int dep_dir, float *hist_point, int ave, Mat *color, int *top){

  #define allow_num 3     //連続性が途絶えた場合の許容回数
  #define allow_range 30  //連続性の幅
  #define allow_hight 0   //開始,終了点の高さ
  int i;
  int j = 0;
  int tmp, max, judge;
  int cnt = 0;
  int check;
  int st, tmp_tp, en;
  int width, hight;
  unsigned int all_area = 0, hist_area = 0;

  //動物体があるか判断
  judge = Judge_Rest(scan_dir, hist_point, ave);

  if (judge == 1) {
    cnt = 0;
  }else if(judge == 0){
    //ノイズの除去
    for (i = 0; i < 10; i++) {
      medianBlur(*color, *color, 3);
    }
    //ヒストグラムを走査
    for(i = 0; i < scan_dir; i++){
      //平均値以上の値が検出された場合，ループ文に入る
      if((int)hist_point[i] > ave + allow_hight){
        st = i; //開始地点
        check = 0;
        //山の登りが連続か判定,allow_num回まで偽判定を許容(条件1)
        for(tmp = 0; tmp < allow_num; tmp++){
          while(hist_point[i] < hist_point[i + allow_range]){
            tmp = 0;
            check = 1;
            i += allow_range;
            //範囲を超えたら強制終了
            if(i > scan_dir){
              check = 0;
              break;
            }
          }
          if(check == 0){
            i += allow_range;
          }
        }
        tmp_tp = i;  //山の頂点

        //条件1を満たした場合下りの連続性判定を行う(条件2)
        if(check == 1){
          for(tmp = 0; tmp < allow_num; tmp++){
            while(hist_point[i] > hist_point[i + allow_range]){
              tmp = 0;
              //ヒストグラムが0の地点を終点とする
              if(hist_point[i + allow_range] < ave + allow_hight){
                check = 2;
                tmp = allow_num + 1;
                break;
              }
              i += allow_range;
              //範囲を超えれば強制終了
              if(i > scan_dir){
                check = 0;
                break;
              }
            }
          }
          en = i;  //終了点
          if(tmp_tp == en){
            check = 0;
          }
        }

        //ヒストグラムの最低面積を考える(条件3)
        if(check == 2){
          width = en - st; //山の幅
          hight = HISTSIZE;
          all_area = width * hight;
          for(i = st; i < en; i++){
            hist_area += (int)hist_point[i];
          }
        }

        //条件1,2を満たしていれば山の数をカウントし，描写
        if(check == 2 && hist_area > (int)(all_area / 10) * 3){
          cnt++;
          top[j] = tmp_tp;
          j++;
        }
      }
    }

    //物体があればその位置に線を描画
    for (i = 0; i < cnt; i++) {
      if(scan_dir > dep_dir){
        cv::line(*color, Point(top[i], 0), Point(top[i], dep_dir), Scalar(0,255,0), 2, 4);
      }else if(scan_dir < dep_dir){
        cv::line(*color, Point(0, top[i]), Point(dep_dir, top[i]), Scalar(255,0,100), 2, 4);
      }
    }
  }
  //フレームに個数を描写
  if(scan_dir > dep_dir){
    char width_num[128];
    sprintf(width_num, "%d", cnt);
    putText(*number, "Number of cols object", Point(20,300), FONT_HERSHEY_TRIPLEX, 1.1, Scalar(0,255,50), 1, CV_AA); //(20,100)の位置に大きさ2、太さ1の青文字で描画
    putText(*number, width_num, Point(200,400), FONT_HERSHEY_TRIPLEX, 2, Scalar(0,255,50), 2, CV_AA); //(20,100)の位置に大きさ2、太さ1の青文字で描画
  }else if(scan_dir < dep_dir){
    char hight_num[128];
    sprintf(hight_num, "%d", cnt);
    putText(*number, "Number of rows object", Point(20,100), FONT_HERSHEY_TRIPLEX, 1.1, Scalar(255,0,0), 1, CV_AA); //(20,100)の位置に大きさ2、太さ1の赤色文字で描画
    putText(*number, hight_num, Point(200,200), FONT_HERSHEY_TRIPLEX, 2, Scalar(255,0,0), 2, CV_AA); //(20,100)の位置に大きさ2、太さ1の赤色文字で描画
  }
  return cnt;
}
//円の描画
void Circle_Draw(Mat *color, int *top1, int *top2, int cnt1, int cnt2){
  for(int i = 0; i < cnt1; i++){
    for(int j = 0; j < cnt2; j++){
      cv::circle(*color, cv::Point(top1[i], top2[j]), 100, cv::Scalar(0,0,200), 3, 4);
    }
  }
}
//最大の頂点の探索
int Max_Search(int *top, int cnt){

  int i, tmp;

  for (i = 0; i < cnt - 1; i++) {
    tmp = top[i];
    if(tmp < top[i + 1]){
      tmp = top[i + 1];
    }
  }
  return tmp;
}
//静止状態かの判別関数
int Judge_Rest(int scan_dir, float *hist_point, int ave){
//goll代数 >> 判定の関数
  int i;
  int num = 0;

  for(i = 0; i < scan_dir; i++){
    //白画素数が平均の半分以上のものをカウント
    if(hist_point[i] > (int)(ave / 10) * 5){
      num++;
    }
  }
  //全体の7割以上が平均の半分より大きければ1を返す
  if(num > (int)(scan_dir / 10) * 7){
    return 1;
  }else{
    return 0;
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
}
