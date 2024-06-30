# 一、代码作用<br>
## src：<br>
  ___gluon_interface___: 调用gluon机械臂api进行初始化，初始化机械臂有关ros话题，并创建线程进行定频率更新机械臂关节角度状态<br>
  ___dual_bringup___: 调用gluon_interface初始化机械臂，订阅控制量话题并调用api执行，创建线程定频率发布joint_state话题以便rviz等可视化工具调用<br>
  ___can_interface___: 接受usbtocan的消息帧，定义pid等控制算法，并创建线程进行定频率更新机械臂关节角度状态<br>
  ___can_bringup___: 调用can_interface初始化6020，订阅控制量话题并调用api执行，定频率发布joint_state话题以便rviz等可视化工具调用<br>
  ___gluonControl_withkey___: 使用键盘控制真实机械臂（不是在控制量话题发布消息控制，而是直接调用gluon api，所以不要和其他控制程序同时用）<br>
  ___gravityoff___: 进行重力补偿，可以人为拖动机械臂并录制机械臂关节角度数据到txt文件<br>
  ___dual_replay___: 播放txt文件中录制的机械臂关节数据，从而复现刚才gravityoff中进行的操作<br>
  ___left_moveit_server___: 使用moveit控制真实机械臂时需要的server，来接收moveit下发的action控制轨迹并执行（发布消息到控制量话题）<br>
  ___right_moveit_server___: 与left同理<br>
  ___rate___: 控制频率的实用工具<br>
  ___teaching___: 录制播放关节数据的实用工具<br>
  ___kinematics_dynamics___: 重力补偿的实用工具<br>

  ## scripts: <br>

  ### moveit: <br>
  
  ___dual_moveitCommander___: 用来测试moveitcommander功能的脚本<br>
  ___movearound___: 创建moveitcommander并使真实机械臂依次移动到特定位置，用来实现手眼标定<br>

  ### serial: <br>

  ___receive_serial___: 通过串口通信接收arduino发来的末端应变片数据，并发布相关ros话题
  
