# 简介

ROS本地计算机对深度学习服务器上Yolact网络结构的远程服务请求的一个简单Demo

![img](misc/display.gif)

# 使用

## 依赖(ROS本地)

- ROS kinetic + Ubuntu 16.04
- [rosbridge](https://github.com/RobotWebTools/rosbridge_suite): 本地ROS计算机上安装

    ```$xslt
    sudo apt-get install ros-<rosdistro>-rosbridge-server
    ```

## 依赖(深度学习服务器)

- **[roslibpy](https://github.com/gramaziokohler/roslibpy)**:

    ```$xslt
    pip install roslibpy
    ```
    
- [Yolact](https://github.com/dbolya/yolact) 以及其依赖

- base64

## 运行

1. [ROS本地] 启动rosbridge_server

    ```$xslt
    roslaunch rosbridge_server rosbridge_websocket.launch
    ``` 
    
    rosbridge_server不要放到launch文件中一起执行, 并保持rosbridge_server不能断连, 由于roslibpy的问题, rosbridge_server一旦断连再重连不能重新连接, 只能重启服务端的程序


2. [服务器端] 将`./yolact_remote/`的脚本放到Yolact工程的根目录, 运行Demo:

    ```$xslt
    cd ${YOLACT_ROOT_PATH}
    python ros_server.py --host --port --img_height --img_width --srv_name --srv_type --model_pth --cuda_device
    ```
    
    - 请把`host`填写为ros master的IP;
    
    - `port`为rosbridge_server监听端口, 默认9090, 在启动rosbridge_server时可以修改
    
    - `srv_name`和`srv_type`填写和ROS本地节点程序一致.
    
2. [ROS本地] 编译`CoCoMaskAPI`, `semantic_msgs`和`yolact_client`, 启动及服务请求节点Demo:

    ```$xslt
    roslaunch yolact_client yolact_tum_demo.launch
    ```
       
    - `yolact_tum_demo.launch`预先设置好TUM数据集序列的路径, 以及`coco.names`的路径
    
    - 输入图像主题是`/image`; 
    
        订阅图像主题是`/image/compressed`; 
        
        实例分割数据结果主题是`/result`; 
        
        可视化主题是`/image_visual`; 
        
        服务名是`/run_yolact`
    
3. [ROS本地] 查看实例分割的结果:
   
    ```$xslt
    rosrun image_view image_view image:=/image_visual
    ```
    
    画图直接放到订阅的回调函数里了, 不要的话launch文件里`visual`设为false就行, 或者函数里抽出来, 独立写成一个节点订阅`/result`主题
    
# 进一步

## 已知问题

1. 速度看脸, 有线网稳定能保持在10Hz上, 无线网不太稳定的时候可能会卡, 最好用5GHz的Wifi

2. 服务器端网络初始化后处理第一帧可能有一个1秒的卡顿, 网络初始化的问题, 先送了一个空白数组冲一下

3. 由于roslibpy的问题, rosbridge_server一旦断连再重连不能重新连接, 只能重启服务端的程序. 所以在运行的时候要保持rosbridge_server的连接

## 其他网络

检测, 语义分割, 实例分割输出的数据结构并不一样. 我还没想到怎么做到统一起来, 目前只能在现有程序上修改

### ROS本地

- 主要需要修改调用服务的类型, 在`semantic_msgs`中已经有一些定义的消息和服务类型, 可以直接使用

### 服务器

- 根据不同网络, 需要一个网络预加载的函数和一个单次推理函数, 然后按对应消息格式组装dict

# 参考

1. [COCO API](https://github.com/cocodataset/cocoapi)
2. [Yolact](https://github.com/dbolya/yolact)
3. [roslibpy](https://github.com/gramaziokohler/roslibpy)
