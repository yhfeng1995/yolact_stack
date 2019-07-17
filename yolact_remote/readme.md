# Overview

Simple remote server demo of Yolact. Put scripts into root directory of [Yolact](https://github.com/dbolya/yolact).

# Usage

## Dependency

- **[roslibpy](https://github.com/gramaziokohler/roslibpy)**;
- [Yolact](https://github.com/dbolya/yolact) and its dependency;
- base64

```$xslt
pip install roslibpy
```

## Run

1. on deep learning server:

    ```$xslt
    cd ${YOLACT_ROOT_PATH}
    python ros_server --host --port --srv_name --srv_type --model_pth --cuda_device
    ```

2. On local compute, **firstly start rosbridge server**, then start your own call service node:

    ```$xslt
    roslaunch rosbridge_server rosbridge_websocket.launch
    ...
    ```

# reference

1. [Yolact](https://github.com/dbolya/yolact)
2. [roslibpy](https://github.com/gramaziokohler/roslibpy)