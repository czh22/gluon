#include "../include/kinematics_dynamics.h"

#include <filesystem>
#include <fstream>
#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl_parser/kdl_parser.hpp>

KinematicsDynamics::KinematicsDynamics(std::string urdf_path)
{
  // 构建机械臂抽象模型
  // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), generateFrame({ 0.0, 0.0, 0.11, 1.5707963267949, 0.0, 0.0 })));
  // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), generateFrame({ -0.27, 0.0, 0.0, 0.0, 0.0, 0.0 })));
  // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), generateFrame({ 0.27, 0.06, 0.0, 0.0, 0.0, 0.0 })));
  // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ), generateFrame({ 0.067, 0.0, -0.00100000000000026, 0.0, 0.0, 0.0 })));
  // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY), generateFrame({ 0.1749, 0.0, 0.0, 1.5707963267949, 0.0, 0.0 })));
  // chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX), generateFrame({ 0.0, 0.0, 0.0, -3.1415926535898, 0.0, 0.0 })));

  // 通过读取urdf，构造chain
  KDL::Tree tree;
  kdl_parser::treeFromFile(urdf_path, tree);
  bool exit_value = tree.getChain("base_link", "link6", chain);

  // 生成运动学求解器
  fksolver_pos = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain);
  iksolver_vel = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain);
  iksolver_pos = std::make_shared<KDL::ChainIkSolverPos_NR>(chain, *fksolver_pos, *iksolver_vel, 100,
                                                            1e-6);  // Maximum 100 iterations, stop at accuracy 1e-6
  // 生成动力学求解器
  idsolver_tor = std::make_shared<KDL::ChainIdSolver_RNE>(chain, KDL::Vector(0.0, 0.0, -9.8));

  // 初始化关节变量 (6个关节)
  joint_angle_states = KDL::JntArray(6);
  new_joint_angles = KDL::JntArray(6);
  joint_velocities = KDL::JntArray(6);
  new_joint_velocities = KDL::JntArray(6);
  joint_acclerations = KDL::JntArray(6);
  ik_results = KDL::JntArray(6);

  // 得到初始末端状态 (init_frame)
  int fk_status = fksolver_pos->JntToCart(joint_angle_states, init_frame);
  end_effector_pose = init_frame;

  // 创建线程
  std::thread thread_1(&KinematicsDynamics::updateStates, this);
  thread_1.detach();
}

KDL::Frame KinematicsDynamics::generateFrame(std::vector<double> pose)
{
  KDL::Vector vector = KDL::Vector(pose[0], pose[1], pose[2]);        // x, y, z
  KDL::Rotation rot = KDL::Rotation::RPY(pose[3], pose[4], pose[5]);  // roll, pitch, yaw
  return KDL::Frame(rot, vector);
}

std::vector<double> KinematicsDynamics::solveFK(std::vector<double> joint_angles)
{
  // 将函数传入参数封装成KDL格式
  KDL::JntArray kdl_joint_input = KDL::JntArray(6);
  for (int i = 0; i < 6; i++)
  {
    kdl_joint_input(i) = joint_angles[i];
  }

  // 定义输出变量
  KDL::Frame kdl_frame_output;

  // 计算正解
  int fk_status = KinematicsDynamics::fksolver_pos->JntToCart(kdl_joint_input, kdl_frame_output);

  // 把kdl_frame_output转化成x、y、z、roll、pitch、yaw
  double x = kdl_frame_output.p.data[0];
  double y = kdl_frame_output.p.data[1];
  double z = kdl_frame_output.p.data[2];
  double roll, pitch, yaw;
  kdl_frame_output.M.GetRPY(roll, pitch, yaw);

  // 将x、y、z、roll、pitch、yaw封装成vector
  std::vector<double> result;
  result.push_back(x - init_frame.p.data[0]);
  result.push_back(y - init_frame.p.data[1]);
  result.push_back(z - init_frame.p.data[2]);
  double init_roll, init_pitch, init_yaw;
  init_frame.M.GetRPY(init_roll, init_pitch, init_yaw);
  result.push_back(roll - init_roll);
  result.push_back(pitch - init_pitch);
  result.push_back(yaw - init_yaw);

  return result;
}

std::vector<double> KinematicsDynamics::solveIK(std::vector<double> end_effector_pose)
{
  double x = end_effector_pose[0];
  double y = end_effector_pose[1];
  double z = end_effector_pose[2];
  double roll = end_effector_pose[3];
  double pitch = end_effector_pose[4];
  double yaw = end_effector_pose[5];

  // 封装机械臂末端期望位姿
  KDL::Vector vector = KDL::Vector(x, y, z) + init_frame.p;
  KDL::Rotation rot = KDL::Rotation::RPY(roll, pitch, yaw);
  KDL::Frame frame_target = KDL::Frame(rot, vector);

  // 计算逆解
  int ik_status = iksolver_pos->CartToJnt(joint_angle_states, frame_target, ik_results);

  // 封装逆解结果为Vector
  std::vector<double> result = { pi2pi(ik_results(0)), pi2pi(ik_results(1)),  pi2pi(ik_results(2)),
                                 pi2pi(ik_results(3)), pi2pi(-ik_results(4)), pi2pi(ik_results(5)) };

  return result;
}

void KinematicsDynamics::updateJointStates(std::vector<double> set_joint_angles)
{
  for (int i = 0; i < 6; i++)
  {
    new_joint_angles(i) = set_joint_angles[i];
  }
}

void KinematicsDynamics::updateStates()
{
  // 线程频率控制
  Rate rate(thread_frequency);
  while (1)
  {
    // 更新关节状态
    for (int i = 0; i < 6; i++)
    {
      // new_joint_velocities(i) = (new_joint_angles(i) - joint_angle_states(i)) / (1.0 / thread_frequency);
      // joint_acclerations(i) = (new_joint_velocities(i) - joint_velocities(i)) / (1.0 / thread_frequency);
      // joint_velocities(i) = new_joint_velocities(i);
      joint_angle_states(i) = new_joint_angles(i);
    }
    // 更新末端执行器位姿
    end_effector_pose = generateFrame(solveFK(
        { joint_angle_states(0), joint_angle_states(1), joint_angle_states(2), joint_angle_states(3), joint_angle_states(4), joint_angle_states(5) }));

    // 频率控制
    rate.sleep();
  }
}

std::vector<double> KinematicsDynamics::solveID(std::vector<double> end_effector_force)
{
  KDL::Vector force = KDL::Vector(0, 0, 0);
  KDL::Vector torque = KDL::Vector(0, 0, 0);
  KDL::Wrenches wrenches;
  for (int i = 0; i < 6; i++)
  {
    wrenches.push_back(KDL::Wrench());
  }

  std::vector<double> froce_torque = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  /*
  TODO: 根据目标点计算力矩
  */
  wrenches[5].force.data[0] = -froce_torque[0];
  wrenches[5].force.data[1] = -froce_torque[1];
  wrenches[5].force.data[2] = froce_torque[2];

  wrenches[5].torque.data[0] = -froce_torque[3];
  wrenches[5].torque.data[1] = froce_torque[4];
  wrenches[5].torque.data[2] = froce_torque[5];

  // 进行逆动力学解算
  KDL::JntArray results = KDL::JntArray(6);
  idsolver_tor->CartToJnt(joint_angle_states, joint_velocities, joint_acclerations, wrenches, results);

  // 封装结果
  std::vector<double> result = { results(0), results(1), results(2), results(3), results(4), results(5) };
  return result;
}

double KinematicsDynamics::pi2pi(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}