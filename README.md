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

  ___catch_demo___: yolov8检测定位快递盒并通过moveit控制机械臂抓取的演示demo代码<br>

  ### moveit: <br>
  
  ___dual_moveitCommander___: 用来测试moveitcommander功能的脚本<br>
  ___movearound___: 创建moveitcommander并使真实机械臂依次移动到特定位置，用来实现手眼标定<br>

  ### serial: <br>

  ___receive_serial___: 通过串口通信接收arduino发来的末端应变片数据，并发布相关ros话题<br>

  ### camera: <br>
  ___Camera_Calibratet_Distort___: 用来标定相机内参<br>
  ___take_photo___: 使用键盘当快门进行拍照（用来进行手眼标定）<br>
  ___calibrate___: 用来标定相机外参（手眼标定）<br>
  ___image_process_py___: 接收相机话题并使用yolov8进行快递盒检测<br>
  ___edge___: 一些测试代码，为了使得yolov8检测效果更好<br>

  ### gazebo: <br>
  ___gazebo_commander___: 键盘控制gazebo中的仿真机械臂<br>
  ___random_gazebo___: 使仿真机械臂进行随机移动，之前用来获取训练数据，但这项工作已经被弃用<br>
  ___realtime_joint___: 将joint消息中的时间戳从仿真时间改为ros时间，同上也已经弃用<br>
  ___record_gazebo_data___: 记录仿真中机械臂的各种数据用来训练模型，同上也已经弃用<br>
  ___gazebo_test___ 和 ___model_operation___: 同上已经弃用<br>

  ___gazebo_test2___: 使用imu对仿真机械臂进行遥操作<br>

  ### imu: <br>
  ___imu_calibrate___: 之前用来测试将imu当前位置标定为0初始位置的代码，成功后已经整合进其他代码中<br>
  ___imu_transform___: 用来补偿掉imu加速度数据中的重力成分，这项工作已经弃用<br>
  ___speed_integration___: 用来将imu中的加速度积分为速度，同上已经弃用<br>
  ___position_integration___: 用来将速度积分为位置，同上已经弃用<br>
  ___imu_control_real___: 用来使用imu控制真实机械臂进行遥操作，与gazebo中的gazebo_test2对应，是其通过测试后应用到真实机械臂的版本<br>
  ___test4___: 一些测试代码<br>

  
  
  
