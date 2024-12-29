# Final Project of General Robot @ SJTU

## TODO
- [x] 1. 2D像素空间 -> 机器人空间 基础实现（using cv2）
- [x] 2. 机械臂仿真，ik，motion planning（using pybullet）
- [x] 3. 机械臂控制（sdk 2.1.14），402部分机械臂无法网线连接，注意甄别！
- [ ] 4. 接入ModelArts推理
- [ ] 5. 2D像素空间 -> 机器人空间 优化

## 接入ModelArts推理
python调用modelarts api请参考APIGW-python-sdk-2.0.5/main.py  
参考其实现test_planning_real.py中RobotController类的get_pixel_point()函数

## 项目入口
test_planning_real.py 中包含仿真测试和真机测试函数