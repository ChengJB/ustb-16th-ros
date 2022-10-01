# 第十六届智能车校内赛ROS组竞赛

第一次分站赛下发文件

## 在本地获得一个本工作空间的副本

首先安装git

```sh
sudo apt install git 
```

在一个合适的文件夹内, 克隆本仓库

```sh
git clone https://github.com/ChengJB/ustb-16th-ros.git
```

或者你可以使用GitHub页面上的下载功能, 然后解压到合适位置

## 构建这个工作空间

相信你已经配置完成ROS的开发环境了  
在`ustb-16th-ros'这个目录下运行

```sh
catkin_make
```

就可以构建这个项目  
你可能会因为缺少必要的包而失败, 可以根据报错信息来找到你需要安装哪个包  
比如, 你可能需要运行

```sh
sudo apt update
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-gmapping
sudo apt install ros-noetic-dwa-local-planner
```

来安装ros的导航套件, gmapping以及dwa规划器  

> 如果你使用的是ubuntu18.04+ros melodic, 你可能需要替换上述命令中的noetic为melodic

重复这样的步骤, 直到安装了所有依赖的包, 再次执行

```sh
catkin_make
```

应该就可以成功构建了

别忘了source当前的工作空间

```sh
source devel/setup.bash
```

## 目录结构介绍

本项目包含三个功能包, 处于src文件夹中

```txt
 .
├──  src
│  ├──  ucar_bringup     # 包含启动项目的总launch文件的功能包
│  ├──  ucar_navigation  # 包含导航的功能包
│  ├──  gazebo_pkg       # 包含仿真环境的功能包
```

他们的功能写在上面

在 `ucar_bringup` 功能包中, 提供了3个示例的launch文件

- example.launch, 会启动示例地图的仿真环境, 启动导航框架. 在rviz中使用2d_nav_goal就可以让小车导航了
- nav_start.launch, 会启动比赛地图的仿真环境, 启动导航框架. 此时导航框架中仍是示例环境的地图, 小车不能正确导航

> 需要你自己建立比赛环境的地图  
> 此仿真环境中不包含障碍物, 带有障碍物的地图将在分站赛前3天下发

- mapping.launch, 会启动比赛地图仿真环境, 启动建图节点

> 为了完成建图, 需要你手动控制小车跑完赛道, 比如使用 [teleop_twist_keyboard](http://wiki.ros.org/teleop_twist_keyboard)  
> 当得到满意的地图后, 运行 `rosrun map_server map_saver' 来把建好的地图保存到当前目录.

## 算法与参数

本工作空间使用了move_base导航框架  
启动于 `src/ucar_navigation/launch/move_base.launch`  
配置文件在 `src/ucar_navigation/config/move_base_params.yaml`  
比如要更改路径规划算法, 更改更新频率等, 你会需要修改这个文件

  

本示例使用了dwa作为导航算法,  
根据上面的提示, 你应该已经能找到它的launch与参数了,  
你将通过调试他来提高小车的性能  
你可以尝试将其更换为其他算法, 比如teb_local_planner

  

建图的也是同理, 这里使用了gmapping, 或许尝试将其换成hector或者cartographer会得到更好的结果

在 `src/ucar_navigation/config/' 下还有很多重要的参数, 比如代价地图相关的, 请尝试调试它们
## 问题补充
![image](https://user-images.githubusercontent.com/79188319/193418829-fa0789df-b112-45e7-8b53-59ae4c0f7d07.png)
如果遇到以上gazebo卡住的界面，请把群里的 models.zip 文件解压后放到根目录下 .gazebo 里面，如下图所示
![image](https://user-images.githubusercontent.com/79188319/193418848-d91b9936-a7ed-4080-a358-ae48064b01a2.png)

（注：只要一个models文件夹即可，不要多个models文件夹嵌套）

