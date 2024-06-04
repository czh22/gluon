#ifndef KINEMATICS_DYNAMICS_H_
#define KINEMATICS_DYNAMICS_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainidsolver.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/tree.hpp>
#include <kdl/frames_io.hpp>

#include <vector>
#include <thread>
#include <memory>

#include "rate.h"

class KinematicsDynamics
{
public:
  KinematicsDynamics(std::string urdf_path);
  ~KinematicsDynamics() = default;

  // 运动学正解，传入六个关节的角度，返回末端执行器的位姿
  std::vector<double> solveFK(std::vector<double> joint_angles);

  // 运动学逆解，传入末端执行器的位姿，返回六个关节的角度
  std::vector<double> solveIK(std::vector<double> end_effector_pose);

  // 动力学逆解，传入末端执行器的受力，返回六个关节的力矩
  std::vector<double> solveID(std::vector<double> end_effector_force);

  // 更新关节状态
  void updateJointStates(std::vector<double> set_joint_angle_states);

private:
  // 角度约束
  double pi2pi(double angle);

  // 机械臂抽象模型
  KDL::Chain chain;

  // 构建kdl的Frame
  KDL::Frame generateFrame(std::vector<double> pose);

  // KDL运动学求解器
  std::shared_ptr<KDL::ChainFkSolverPos> fksolver_pos;
  std::shared_ptr<KDL::ChainIkSolverVel> iksolver_vel;
  std::shared_ptr<KDL::ChainIkSolverPos> iksolver_pos;
  // KDL动力学求解器
  std::shared_ptr<KDL::ChainIdSolver_RNE> idsolver_tor;

  // 机械臂初始位置
  KDL::Frame init_frame;

  // 关节状态
  KDL::JntArray joint_angle_states;
  KDL::JntArray new_joint_angles;
  KDL::JntArray joint_velocities;
  KDL::JntArray new_joint_velocities;
  KDL::JntArray joint_acclerations;
  // 末端执行器状态
  KDL::Frame end_effector_pose;

  // 逆解结果
  KDL::JntArray ik_results;

  // 创建独立线程
  pthread_t thread_1;
  // 线程函数，以一定频率更新关节状态
  void updateStates();
  // 线程运行频率
  double thread_frequency = 100.0;
};

#endif