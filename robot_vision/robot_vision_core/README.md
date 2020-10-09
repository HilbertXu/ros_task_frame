# Target Tracker 相关功能的使用说明

### 1. 可以跟踪哪些目标

目前在robot_vision_core/src/target_tracker.cpp中已经实现了跟踪由openpose，yolo，opencv-face-detection，opencv-face-recognition，opencv-color-detection提供的识别目标。

其中：

1. openpose：可以用来跟踪挥左手（waving_left），挥右手（waving_right），举起双手（surrender），站立两手都不举起来（standing）的人。需要先通过roslaunch启动openpose_ros.launch后再使用。

   使用说明：在target_tracker.launch中设置

   ```xml
   <arg name="target_tracker/track"  default="openpose" />
   <!-- 四种姿势中选一种即可 -->
   <arg name="target_tracker/target" default="waving_left/waving_right/surrender/standing" /> 
   ```

   

2. yolo：可以用来跟踪yolov3能识别出来的所有物体（详见robot_vision_openvino/models/yolov3/yolov3.names文件内容，注意该文件内容与模型输出结果为一一映射关系，不可随便修改）

   使用说明：在target_tracker.launch中设置

   ```xml
   <arg name="target_tracker/track"  default="yolo" />
   <!-- yolov3.names中选一种即可 -->
   <arg name="target_tracker/target" default="bottle" />
   ```

   

3. opencv-face-detection：可以跟踪识别到的第一张人脸

   使用说明：在target_tracker.launch中设置

   ```xml
   <arg name="target_tracker/track"  default="face" />
   <!-- 此处第二个参数没有作用，默认跟踪opencv检测到的第一张人脸 -->
   <arg name="target_tracker/target" default="0" />
   ```

   

4. opencv-face-recognition：可以跟踪识别出来的指定人脸

   使用说明：在target_tracker.launch中设置

   ```xml
   <arg name="target_tracker/track"  default="person" />
   <arg name="target_tracker/target" default="<person_name>" />
   ```

   

5. opencv-color-detection：可以跟踪由opencv框出来的指定的颜色区域

   使用说明：在target_tracker.launch中设置

   ```xml
   <arg name="target_tracker/track"  default="color" />
   <!-- 此处第二个参数没有作用，默认跟踪认为框定的颜色区域 -->
   <arg name="target_tracker/target" default="0" />
   ```

   

### 2. 节点参数初始化逻辑

在当时开发的时候，为了后续开发的方便，我考虑为target_tracker.cpp设置三种模式：

1. 第一种**受控模式（under_control=true）**，该模式下节点初始化时不会设定track和target参数，等待控制节点通过topic传入这两个参数

2. 第二种是**非受控模式（under_control=false）**：

   1. 参数由控制台传入，则启动指令为：

      ```shell
      rosrun robot_vision_core target_tracker -track <yolo/openpose/face/person/color> -target <target-name>
      ```

   2. 参数由launch文件传入，在launch文件中修改：

      ```xml
      <arg name="target_tracker/track"  default="<yolo/openpose/face/person/color>" />
      <arg name="target_tracker/target" default="<target-name>" />
      ```



### 3. 控制摄像头跟踪目标的逻辑

1. 电机控制逻辑：

   每次需要同步地接收到来自<yolo/openpose/opencv-*>的目标识别结果（目标识别框的像素坐标），以及pan motor和lift motor的电机状态。将两个电机的当前状态，加上目标识别框的像素坐标在x方向和y方向上和图像中心的差值乘上较小的缩放因子（0.001），得到电机当前时刻应该转到的状态，然后通过topic的方式控制电机转到目标状态。

   ```cpp
   void TargetShift::dynamixelControl(int curr_x, int curr_y, float pan_state, float lift_state, float scale) {
     int error_x = centerX_ - curr_x;
     int error_y = centerY_ - curr_y;
     printf("X error: %d, Y error: %d\n", error_x, error_y);
     turnedAngle_ = pan_state + error_x*scale;
   
     float next_state_pan =  pan_state + error_x*scale;
     float next_state_lift = lift_state - error_y*scale;
     printf("Current pan: %f, lift: %f -> Next pan: %f, lift: %f\n",pan_state, lift_state, next_state_pan, next_state_lift);
     
     std_msgs::Float64 msg_pan;
     std_msgs::Float64 msg_lift;
   
     msg_pan.data = next_state_pan;
     msg_lift.data = next_state_lift;
   
     headPanJointPublisher_.publish(msg_pan);
     headLiftJointPublisher_.publish(msg_lift);
   }
   
   ```

   

2. 初始搜索目标逻辑：

   当整套系统启动时，如果在头部摄像机的视野范围内没有检测到跟踪目标，则开始进行搜索。在搜索过程中，头部摄像机会先保持平视，转动一圈，如果在这一圈的过程中没有检测到目标物体，则会调整lift pan让摄像机朝下开始重新转动一整圈来搜索目标。一旦在相机视野范围内检测到目标物体，则给两个电机发布当前的状态作为目标状态，来使电机停止转动，并进入跟随过程

   涉及这一过程的代码块为：

   ```cpp
   int targetIndex_ = 0;
     for (int i = 0; i < msg->bounding_boxes.size(); i++) {
       if (msg->bounding_boxes[i].Class == targetName_) {
         // 如果检测到目标，记录目标的index，并将FLAG_target_found置为true
         targetIndex_ = i;
         FLAG_target_found = true;
         FLAG_start_track = true;
         dynamixelControl(pan_state->current_pos, lift_state->current_pos);
         break;
       }
     }
     // 如果没有检测到目标
     if (!FLAG_target_found) {
       if (searchTargetCount_ == 3) {
         // 在平视状态下头部已经转过一周，设置头部向下观察
         if (fabs(lift_state->goal_pos - 0.43) > 0.1) {
           // 只发布一次lift关节控制指令
           dynamixelControl(0.0, 0.43);
         }
         // 等待lift关节完成移动，然后将搜索次数置0
         if (fabs(lift_state->current_pos - lift_state->goal_pos) <= 0.1) {
           searchTargetCount_ = 0;
         }
       }
       // 没有检测到目标时，转动头部搜索目标
       if ((fabs(pan_state->current_pos - pan_state->goal_pos) <= 0.1) && (searchTargetCount_ < 3)) {
         ROS_INFO("[TargetTracker] Searching for target: %s ...", targetName_.c_str());
         searchTargetCount_ += 1;
         // 此时舵机并不在运动中
         /* 
          * searchFrameCount： 1 -> pan_angle: -2.610
          * searchFrameCount: 2 -> pan_angle: 2.610
          */
         float pan_angle = pow((-1), searchTargetCount_)*(searchTargetCount_%3)*(2.610);
         dynamixelControl(pan_angle, lift_state->current_pos);
       }
     
   ```

   

3. 跟踪逻辑：

   在跟踪过程中我们需要明白什么时候已经完成了跟随过程，需要调整底盘对准物体。目前我使用的方法是，如果物体当前时刻识别框中心到摄像机画面中心的距离少于10个像素，则认为当前时刻物体没有发生运动，静止帧计数+1，一旦物体的识别框中心到画面中心的偏移量超过10个像素，则认为物体发生了运动，将静止帧计数器清零。只有当物体连续6帧没有发生运动时，才认为物体已经静止，开始调整底盘对准物体。

   **目前存在的问题**：识别框或者识别骨架跳动幅度较大，无法完成6帧的累积计数，导致机器人认为目标物体一直处在运动中，今儿无法进入下一个环节。已经尝试采用了均值滤波的方法来缓解这个情况，但是效果一般。

### 

### 4. Target Tracker和其他功能的结合

1. 和机器人底盘的结合

   在完成目标跟踪过程之后，此时大部分情况下摄像机与机器人之间存在角度的偏差（记为$\theta$），这个时候需要通过MoveRobotAction来控制机器人转过-$\theta$角来使机器人底盘和头部摄像机的朝向一致。这一部分中，利用了机器人的里程计odometry来获取较为精确的机器人转动角度信息。

   **可能存在的问题**：在调整底盘的过程中，可能出现底盘转动速度过快导致视野内目标丢失的情况，目前没有很好的解决方案。可以考虑根据目标识别框中心与图像中心的x，y坐标差值大小调整电机控制过程中的缩放因子scale，因为电机速度好像无法动态地调节，所以很难从电机速度入手来解决这个问题，只能在实际应用中根据目标物体的远近来调整电机的速度。

2. 和TF变换和导航功能的结合

   在完成目标物体的跟踪，并控制底盘和摄像头均对准目标物体后，可以通过摄像机深度坐标系到机器人底盘坐标系或者是地图坐标系（如果需要使用到地图坐标系的变换，则需要加载建好的地图，并且需要amcl算法的粒子群收敛，才能有较为精确的tf变换）的tf变换，得到物体中心点在目标坐标系下的位置。这样一来就可以控制机器人底盘直接逼近目标物体，或者是通过导航节点，控制机器人自主导航到目标物体附近。

**可能用到的代码**：

控制底盘转动的代码在**robot_navigation_core/src/move_robot_server.cpp**

通过tf变换定位物体的代码：**robot_vision_core/src/object_localization.cpp**

接受物体定位坐标并进行导航的代码：**robot_navigation_core/src/navigate_to_target.cpp**



