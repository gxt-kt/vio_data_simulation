//
// Created by hyj on 17-6-22.
//

#include <sys/stat.h>

#include <fstream>

#include "imu.h"
#include "utilities.h"

const std::string HOUSE_FILE = "house_model/house.txt";
const std::string SAVE_POINTS_FILE = "all_points.txt";

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

/*
 * 传入的文件，每一行代表两个点，前三个是一组xyz，后三个是一组xyz
 * 然后把所有点不重复的插入到points中
 * 并且每一行的两个点组成一个边，插入到边中
 * 然后就把所有的点保存到文件save_points_file中
 */
void CreatePointsLines(std::string file, std::string save_points_file,
                       Points& points, Lines& lines) {
  std::ifstream f;
  f.open(file);

  while (!f.eof()) {
    std::string s;
    std::getline(f, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      double x, y, z;
      ss >> x;
      ss >> y;
      ss >> z;
      Eigen::Vector4d pt0(x, y, z, 1);
      ss >> x;
      ss >> y;
      ss >> z;
      Eigen::Vector4d pt1(x, y, z, 1);

      bool isHistoryPoint = false;
      for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d pt = points[i];
        if (pt == pt0) {
          isHistoryPoint = true;
        }
      }
      if (!isHistoryPoint) points.push_back(pt0);

      isHistoryPoint = false;
      for (int i = 0; i < points.size(); ++i) {
        Eigen::Vector4d pt = points[i];
        if (pt == pt1) {
          isHistoryPoint = true;
        }
      }
      if (!isHistoryPoint) points.push_back(pt1);

      // pt0 = Twl * pt0;
      // pt1 = Twl * pt1;
      lines.emplace_back(pt0, pt1);  // lines
    }
  }

  // 下面的代码只是为了增加观测点，可以完全不用加，加了也行
  // create more 3d points, you can comment this code
  // int n = points.size();
  // for (int j = 0; j < n; ++j) {
  //   Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5, 0.5, -0.5, 0);
  //   points.push_back(p);
  // }

  // save points
  save_points(save_points_file, points);
}

int main() {
  // Eigen::Quaterniond Qwb;
  // Qwb.setIdentity();
  // Eigen::Vector3d omega (0,0,M_PI/10);
  // double dt_tmp = 0.005;
  // for (double i = 0; i < 20.; i += dt_tmp) {
  //     Eigen::Quaterniond dq;
  //     Eigen::Vector3d dtheta_half =  omega * dt_tmp /2.0;
  //     dq.w() = 1;
  //     dq.x() = dtheta_half.x();
  //     dq.y() = dtheta_half.y();
  //     dq.z() = dtheta_half.z();
  //     Qwb = Qwb * dq;
  // }
  // std::cout << Qwb.coeffs().transpose() <<"\n"<<Qwb.toRotationMatrix() <<
  // std::endl;

  // 建立keyframe文件夹
  mkdir("keyframe", 0777);

  // 生成3d points
  Points points;
  Lines lines;
  CreatePointsLines(HOUSE_FILE, SAVE_POINTS_FILE, points, lines);

  // IMU model
  Param params;
  IMU imuGen(params);

  // create imu data
  // imu pose gyro acc
  // imudata 产生的运动轨迹是20s
  // 实际仿出来的位置路径是一个xy组成了椭圆，z呈波浪状
  // x=15cos(PI/10*t)+5
  // y=20sin(PI/10*t)+5
  // z=sin(PI*t)+5
  // 其中t会从0s走到20s
  std::vector<MotionData> imudata;
  std::vector<MotionData> imudata_noise;
  for (float t = params.t_start; t < params.t_end;) {
    MotionData data = imuGen.MotionModel(t);
    imudata.push_back(data);

    // add imu noise
    // 在data的基础上添加噪声
    MotionData data_noise = data;
    imuGen.addIMUnoise(data_noise);
    imudata_noise.push_back(data_noise);

    t += 1.0 / params.imu_frequency;
  }
  imuGen.init_velocity_ = imudata[0].imu_velocity;
  imuGen.init_twb_ = imudata.at(0).twb;
  imuGen.init_Rwb_ = imudata.at(0).Rwb;
  save_Pose("imu_pose.txt", imudata);
  save_Pose("imu_pose_noise.txt", imudata_noise);

  // test the imu data, integrate the data to generate the imu trajecotryimu
  imuGen.testImu("imu_pose.txt", "imu_int_pose.txt");

  imuGen.testImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");

  // cam pose
  std::vector<MotionData> camdata;
  for (float t = params.t_start; t < params.t_end;) {
    MotionData imu =
        imuGen.MotionModel(t);  // imu body frame to world frame motion
    MotionData cam;

    cam.timestamp = imu.timestamp;
    cam.Rwb = imu.Rwb * params.R_bc;  // cam frame in world frame
    cam.twb = imu.twb +
              imu.Rwb * params.t_bc;  //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

    camdata.push_back(cam);
    t += 1.0 / params.cam_frequency;
  }
  save_Pose("cam_pose.txt", camdata);
  save_Pose_asTUM("cam_pose_tum.txt", camdata);

  // points obs in image
  // 总共600副图像，1s30副图像
  for (int n = 0; n < camdata.size(); ++n) {
    MotionData data = camdata[n];
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Twc.block(0, 0, 3, 3) = data.Rwb;
    Twc.block(0, 3, 3, 1) = data.twb;

    // 遍历所有的特征点，看哪些特征点在视野里
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        points_cam;  // ３维点在当前cam视野里
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        features_cam;  // 对应的２维图像坐标
    for (int i = 0; i < points.size(); ++i) {
      Eigen::Vector4d pw = points[i];  // 最后一位存着feature id
      pw[3] = 1;                       // 改成齐次坐标最后一位
      Eigen::Vector4d pc1 =
          Twc.inverse() * pw;  // T_wc.inverse() * Pw  -- > point in cam frame

      if (pc1(2) < 0) continue;  // z必须大于０,在摄像机坐标系前方

      // obs存放的是特征点对应相机归一化平面的点
      Eigen::Vector2d obs(pc1(0) / pc1(2), pc1(1) / pc1(2));
      // if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
      // (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
      {
        // points是特征点在世界坐标系下的点，第四个参数永远为1
        points_cam.push_back(points[i]);
        // obs是特征点在相机归一化平面下的点
        features_cam.push_back(obs);
      }
    }

    // save points
    std::stringstream filename1;
    filename1 << "keyframe/all_points_" << n << ".txt";
    save_features(filename1.str(), points_cam, features_cam);
  }

  // lines obs in image
  // 总共600副图像，1s30副图像
  for (int n = 0; n < camdata.size(); ++n) {
    MotionData data = camdata[n];
    Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
    Twc.block(0, 0, 3, 3) = data.Rwb;
    Twc.block(0, 3, 3, 1) = data.twb;

    // 遍历所有的特征点，看哪些特征点在视野里
    // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
    // points_cam;    // ３维点在当前cam视野里
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        features_cam;  // 对应的２维图像坐标
    for (int i = 0; i < lines.size(); ++i) {
      Line linept = lines[i];

      Eigen::Vector4d pc1 =
          Twc.inverse() *
          linept.first;  // T_wc.inverse() * Pw  -- > point in cam frame
      Eigen::Vector4d pc2 =
          Twc.inverse() *
          linept.second;  // T_wc.inverse() * Pw  -- > point in cam frame

      if (pc1(2) < 0 || pc2(2) < 0) continue;  // z必须大于０,在摄像机坐标系前方

      // obs是线的两点在相机归一化平面上的表示
      Eigen::Vector4d obs(pc1(0) / pc1(2), pc1(1) / pc1(2), pc2(0) / pc2(2),
                          pc2(1) / pc2(2));
      // if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) <
      // params.image_w)
      { features_cam.push_back(obs); }
    }

    // save points
    std::stringstream filename1;
    filename1 << "keyframe/all_lines_" << n << ".txt";
    save_lines(filename1.str(), features_cam);
  }

  return 0;
}
