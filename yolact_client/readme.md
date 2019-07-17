# Overview

Simple client demo of calling remote Yolact service

# Usage

## Dependency

- cv_bridge

- semantic_msgs (first compile)

## Run

1. compile **after** `semantic_msgs`

2. run rosbridge

3. set dataset path and visual flag

4. run demo node

    ```$xslt
    roslaunch yolact_client yolact_tum_demo.launch
    ```

5. if set visual flag, show video:

    ```$xslt
    rosrun image_view image_view image:=/image_visual
    ```