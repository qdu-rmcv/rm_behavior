参照学习深北莫决策架构开源，在此基础上修改为现有代码

## 指南

1.本项目运行依赖 behavior ros2,需要前往 git 拉取后才可以编译运行

```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/BehaviorTree.ROS2.git
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

- [ ] 完成裁判系统内容传入的判断
    - [x] 完成血量和弹丸量判断节点
    - [x] 完成时间检查节点
    - [x] rfid 状态判断
- [x] 完成导航到点
- [x] 完成追击
- [x] 跟随己方机器人

ROS话题的订阅和广播统一定义在 rm_behavior.cpp

在 plugin 中定义行为树节点部分
