/**
*@file    chaser.hpp
*@brief   人追従プログラム ヘッダファイル
*@author  Kosei Demura , Ryoya Yamada
*@date    2017_12_28
*/

// ver1 
// 日付 9_24,yamada
//#define VER1 // ver1に切り替える際はコメントアウト

// ver2 
// 日付 12_28,yamada
// 変更点 変更したメンバ変数,関数にはコメントでver2
//        検出範囲を半径40[cm]の円に変更
//        オドメトリを利用して相対座標を世界座標に変換
//        世界座標系の画像を利用して動体検出や検出範囲外の静止物体を脚候補から除外する
//        動体検出を行い追跡する脚は動体を優先するが動体がない場合はver1と同じように脚候補同士で追跡する
//
//動体検出アルゴリズム
//    脚候補の位置の画像とLIDARから得られた過去の画像を利用して検出している
//    各画像は白(画素値255)で初期化
//        1,オドメトリを利用して相対座標を世界座標系に変換し画像(world_pose_image)に黒(画素値0)で描写する
//          画像座標系を相対座標に変換してから世界座標に変換すると画像上での物体同士が近すぎて重なってしまうので
//          定数(kMagnificationWorldImagePos)倍して重ならないようにする
//        2,world_pose_imageを過去の画像(last_world_pose_image)にkUpdateLastImageCount回のループ毎にクローンする
//        　１時刻前の画像を使わないのは比較してもほぼ差分が取れないため
//          このときの過去の画像にオープニング処理を行ったものがopening_last_world_pose_image
//        3,現在の脚候補の位置を世界座標系に変換しworld_pose_candidate_leg_imageに黒(画素値0)で描写する
//        4,world_pose_candidate_leg_imageとopening_last_world_pose_imageを比較,反転させた差分画像をsubstraction_world_pose_imageとする
//        5,substraction_world_pose_imageに脚候補の座標と同じ座標が黒で描写されていれば動体と判断し画像(lidar_image)赤丸で描写
//        6,追跡する人の位置を推定,追従
//          検出範囲内にある動体,脚候補を利用して追跡する
//          動体2つの中心を優先するが脚は同時に動かないためほとんどの場合動体２つは検出できない
//          動体が2つない場合は動体と一番近い脚候補の中心を追跡する
//          動体がない場合はver1と同じように脚候補同士の中心で追跡する
//
//検出範囲外の静止物体を脚候補から除外する処理
//    動体検出により脚候補が動体か判断する
//    検出範囲外かつ動体でない場合,その脚候補の座標を世界座標系に変換し､static_object_world_pose_imageに黒(画素値0)で描写
//    static_object_world_pose_imageにオープニング処理を行ったものがopening_static_object_world_pose_image
//    脚候補か判断する際､opening_static_object_world_pose_imageが脚候補の座標で黒になっているかを判断する
//    黒であれば脚候補に似た静止物体と判断できるので脚候補の探索から除外する
//    人を見失っている時に画像を初期化する
//
#define VER2 // ver1に切り替える際はコメント

#ifndef DFOLLOW3_HPP
#define DFOLLOW3_HPP

#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sys/time.h>
#include <string>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <boost/regex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/stat.h>
#include <sys/types.h>

#include <time.h>

//#define MOMENT_EXEL// 慣性モーメントデータ取得用
//#define RECORD       //録画用

#define DEG2RAD(DEG) ( (M_PI) * (DEG) / 180.0 )
#define RAD2DEG(RAD) ( 180.0  * (RAD) / (M_PI))

using namespace std;
using namespace cv;

#ifdef VER1 //yamada
int g_find_leg_area = 20;
const int kExtendArea = 25;
const int kOriginalArea = g_find_leg_area;
#endif

#ifdef VER2 //yamada
int g_find_leg_radius = 20 ; // 検出範囲(円)の中心座標からの半径[px],yamada,ver2
const int    kExtendRadius        = 25 ;  // 見失ったときの検出範囲 [px],ver2
const int    kOriginalRadius      = g_find_leg_radius; // ver2
const int    kMagnificationWorldImagePos = 10; // 画像上で物体同士が重ならないようにするための世界座標の倍率,ver2
const int    kUpdateLastImageCount = 30; // 比較する世界座標系の画像を更新するループ回数 one loop 30[ms],ver2
#endif

const double kFollowMaxDistance = 6.0; // 追従距離の最大
const double kFollowDistance    = 0.3; // 人からこの距離で追従する
const double kFollowMinDistance = 0.3; // 追従距離の最小
const double kFollowAngle       = 180; // 探す範囲は正面のこの角度[deg]
const double kGainLinear        = 0.4;  // P制御比例ゲイン（並進0.4）
const double kKp                = 0.3; // PD制御ゲイン(回転0.20) 0.20
const double kKd                = 6.0; // 5.25
const double kLinearMaxSpeed    = 0.7;  // 並進の最大速度[m/s]
const double kTurnMaxSpeed      = 0.5;  // 角速度 最大3.14[rad/s](1.0)
//yamada
const double kDefaultDetectPosX	= 250;  // 検出範囲(円)のxの初期の中心座標,[px]
const double kDefaultDetectPosY = 200;  // 検出範囲(円)のyの初期の中心座標,[px]
const int    kLostTime          = 60 ;  // 人を完全に見失ったと判断するループ回数,one loop 30[ms]
const double kLegBetweenDistance = 0.6; // 人の脚だと判断する脚候補間の距離[m]

const int kImageWidth=500, kImageHeight=500; // [px]

const double kMToPixel = 50; // mをpixelへ変換 1m == 50 pixel, 1 pixel = 2cm

// 初期化時に塗りつぶす
cv::Mat lidar_image(cv::Size(kImageWidth, kImageHeight),
                    CV_8UC3, cv::Scalar::all(255));
cv::Mat lidar_gray_image(cv::Size(kImageWidth, kImageHeight),
                         CV_8U, cv::Scalar::all(255));


// 脚候補の世界座標系の画像,ver2,yamada
cv::Mat world_pose_candidate_leg_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));

// ライダーからのデータを世界座標系にした画像,ver2,yamada
cv::Mat world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));
cv::Mat last_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));
cv::Mat erode_last_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));
cv::Mat opening_last_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));

// 世界座標系の画像の差分画像,ver2,yamada
cv::Mat substraction_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));

// 検出範囲外の脚候補の位置を世界座標系にして描写する画像,ver2,yamada
cv::Mat static_object_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));
cv::Mat erode_static_object_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));
cv::Mat opening_static_object_world_pose_image( cv::Size(500,500), CV_8U, cv::Scalar::all(255));

// lidar画像に縮小処理
cv::Mat lidar_erode_image;

// 色
cv::Scalar red(0,0,255), blue(255,0,0), green(0, 255, 0);

// 録画
cv::VideoWriter writer1("videofile.avi", cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), 30.0,
                        cv::Size(kImageWidth, kImageHeight), true);

enum  Robot_Condition { SAFE = 0, DANGER = 119 };

struct Pose {
    double x;
    double y;
    double z;
    double theta;
};

/**
*@brief   人についてのクラス
*@details 人までの距離や角度の情報をメンバとして宣言
*/
class Object
{
private:
    double x_;
    double y_;
    double z_;
    double theta_;

public:
    Object();
    Pose local; // ローカル座標: ROSに合わせて進行方向がx, 左方向がy
    Pose world; // ワールド座標
    int intensity_min_, intensity_max_; // 反射強度の最小、最大値　[0:255]
    double distance_,last_distance_; // 距離
    double angle_,last_angle_; // 角度[rad/s]
    double radius_; // 半径
    cv::Point2f image_pos_; // 画像座標系での位置
    double getX() {
        return x_;
    }
    double getY() {
        return y_;
    }
    double getZ() {
        return z_;
    }
    double getTheta() {
        return theta_;
    }
    void   setX(double _x) {
        x_ = _x;
    }
    void   setY(double _y) {
        y_ = _y;
    }
    void   setZ(double _z) {
        z_ = _z;
    }
    void   setTheta(double _theta) {
        theta_ = _theta;
    }

    //yamada
    double last_human_x_,last_human_y_; //1時刻前の人の位置
    double image_expect_human_x_,image_expect_human_y_; //予測される人の位置
    double last_linear_speed_; //1時刻前の速度
    cv::Point2f dynamic_image_pos_; //動体と予測した物体の位置,ver2
    bool judge_dynamic_; // 動体か判断,ver2
};

/**
*@brief  ロボットやセンサについてのクラス
*@details LIDARの情報やロボットの情報をメンバとして宣言
*/
class Robot
{
private:
    bool   human_lost_;  //人を見失っているか
    double robot_x_,robot_y_,robot_theta_; // 位置 [m], 向き[rad]
    double laser_distance_[1081]; // hokuyo lidar UTM-30LX
    double laser_last_distance_[1081];
    double laser_intensities_[1081];
    double laser_last_intensities_[1081];

    double last_x_, last_y_, last_theta_; // １時刻前のx, y, theta
    double linear_speed_, angular_speed_; // [m/s],[rad/s]
    geometry_msgs::Twist cmd_vel, zero_cmd_vel;

    // yamada
	// 書き込もうとした座標が画像より大きい場合コアダンプする、そのため初期位置(画像の中心)からの座標にずらす
    // そのときに書き込もうとした座標をずらすためのxとyの値,ver2
    double displace_x_;
    double displace_y_;
    int last_image_count_; // このカウントがkUpdateLastImageCountと同じになればlast_world_pose_imageを更新,ver2

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub; // オドメトリのサブスクライバ,ver2

    ros::Publisher find_str_pub;
    ros::Publisher find_human_pub;
    ros::Publisher lost_human_pub;//yamada
    ros::Subscriber follow_human_sub;
  

  
public:
    Robot();
    Pose   local; // ローカル座標
    Pose   world;
    int    dataCount_;
    double laser_angle_min_, laser_angle_max_;
    void   calcHumanPoseVer2(int object_num, Object *object, Object *human_object);   // 人の位置推定 ver2
	void   calcHumanPoseVer1(int object_num, Object *object, Object *human_object); // 人の位置推定 ver1

    void   changeToPicture(int dataCount, double laser_angle_min,                  // LIDARからの情報を画像に変換,ver2
                           double laser_angle_max, double laser_angle_increment);
    int    checkCollision(double *avoid_angle);                                                       // 障害物チェック
    int    checkCondition(double theta);                                           // 進行できるかできないか
    string follow_command; // follow command
    int    findLegs(cv::Mat input_image, Object *object,                           // 脚候補の探索,ver2
                    cv::Mat display_image, cv::Scalar color,
                    int contour_min, int contour_max, int width_min, int width_max,
                    double ratio_min, double  ratio_max,double m00_min, double m00_max,
                    double m10_min, double m10_max, double diff_x,  double diff_y);
    void   followHuman(cv::Mat input_image);     // 人に追従,ver2
    void   followHuman(cv::Mat input_image, bool move);
    void   followHumanCallback(const std_msgs::String msg);
  
    double getAngularSpeed() {
        return angular_speed_;
    }
    double getLinearSpeed()  {
        return linear_speed_;
    }

    void   init(); // パブリッシャーとサブスクライバーの宣言
    void   laserCallback(const sensor_msgs::LaserScan laser_scan);
    void   odomCallback(const nav_msgs::OdometryConstPtr& msg); // ver2
    void   localToWorld(Pose local_pose, Pose *world_pose ); // ローカル座標をワールド座標に変換
    void   move(double linear, double angular);// 並進速度[m/s]と角速度[rad/s]をパブリッシュする
    void   prepRecord(); // 録画する際のエラーチェック
    void   prepWindow(); //LIDAR画像の処理
    void   record(cv::VideoWriter writer, cv::Mat image);// 録画
    void   setAngularSpeed(double angular);
    void   setLinearSpeed(double linear);
    void   showWindow();
    void   welcomeMessage();

    //yamada
    double followLinearSpeed(const Object &human_obj); // 追従速度の設定
    void   defaultPos(Object *human_obj);              // 検出範囲の初期位置
    void   reduceSpeed(Object *human_obj);          // 減速
    void   writeWorldPoseImages(double point_x, double point_y); // 世界座標系の画像に書き込む,ver2
};

Object human;


#endif
