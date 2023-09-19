# DRL_Path_Planning

This is a DRL(Deep Reinforcement Learning) platform built with Gazebo for the purpose of robot's adaptive path planning.

# Environment

## Software

    Ubuntu 16.04
    ROS Kinect
    Python 2.7.12
    tensorflow 1.12.0

# Document

```
src
   ├─ CMakeLists.txt
   ├─ multi_jackal	// 创建模型的功能包
   └─ tf_pkg	// 关于强化学习路径规划的功能包
      ├─ CMakeLists.txt
      ├─ package.xml
      └─ scripts
         ├─ 10_D3QN_PER_image_add_sensor_empty_world_30m.py	// 无障碍环境训练
         ├─ 10_D3QN_PER_image_add_sensor_empty_world_30m_test.py
         ├─ 10_D3QN_PER_image_add_sensor_obstacle_world_30m.py	// 静态障碍环境训练
         ├─ 10_D3QN_PER_image_add_sensor_obstacle_world_30m_test.py
         ├─ 12_pathplanning.py
         ├─ 14_static_obstacle_pathplanning.py
         ├─ 16_plot_10dynamic_obstacle_pathplanning_result.py
         ├─ D3QN_PER_image_add_sensor_dynamic_10obstacle_world_30m_test.py	//动态障碍环境训练
         ├─ Models.py
         ├─ gazebo_env_D3QN_PER_image_add_sensor_empty_world_30m.py	// 无障碍环境搭建
         ├─ gazebo_env_D3QN_PER_image_add_sensor_empty_world_30m_test.py
         ├─ gazebo_env_D3QN_PER_image_add_sensor_obstacle_world_30m.py	// 静态障碍环境搭建
         ├─ gazebo_env_D3QN_PER_image_add_sensor_obstacle_world_30m_test.py
         ├─ gazebo_env_dynamic_obstacle_10jackal_test.py	// 动态环境搭建
         ├─ pathplaner.py
         └─ saved_networks
```

# 使用



# Error

## catkin_make时报错

**描述：**在编译时报错如下：

```shell
Could not find a package configuration file provided by
  "robot_localization" with any of the following names:

    robot_localizationConfig.cmake
    robot_localization-config.cmake
```

**原因**：缺少这个包

**解决**：安装就行了

```shell
sudo apt-get install ros-melodic-robot-localization
```

诸如此类的报错，解决方法都一样安装相关包就行了。

## 加载模型时报错

**描述**：加载训练好的模型时报错：

```shell
DataLossError (see above for traceback): Checksum does not match: stored 1214729159 vs. calculated on the restored bytes 4272926173
```

**原因**：

* 模型文件损坏
* tensorflow版本问题
* 储存介质出现问题

**解决**：

重新训练

## 报错

**描述**：训练时报错

```shell
ValueError: cannot copy sequence with size 724 to array axis with dimension 364
```

**原因**：

维度不匹配

**解决**：

寻找到sick_lms1xx.urdf.xacro文件，在里面将720改为360

文件路径为：`/opt/ros/melodic/share/lms1xx/urdf/sick_lms1xx.urdf.xacro`

命令行输入：

```shell
sudo gedit sick_lms1xx.urdf.xacro
```

## 训练时报错

**描述**：

```shell
UnknownError (see above for traceback): Failed to get convolution algorithm. This is probably because cuDNN failed to initialize, so try looking to see if a warning log message was printed above.
```

**原因**：
显存不足，将显存按需分配。

**解决**：

在开头添加这么一段

```python
import keras  
from keras.backend.tensorflow_backend import set_session
config = tf.ConfigProto()  
config.gpu_options.allow_growth = True 
sess = tf.Session(config=config)
set_session(sess)
keras.backend.clear_session()
```



>  ROS常用API官方链接：
>
>  http://wiki.ros.org/APIs
