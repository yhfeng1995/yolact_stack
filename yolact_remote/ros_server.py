from yolact_interface import YolactInterface
import roslibpy
import numpy as np
import cv2
import base64
import os


class YolactService:
    def __init__(self, model_pth, srv_name, srv_type, host, port=9090, img_height=480, img_width=640):
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.connect()
        self.model = YolactInterface(model_pth)  # TODO: Change load model function
        self.srv_name = srv_name
        self.srv_type = srv_type
        self.h = img_height
        self.w = img_width
        init_input = np.zeros([self.h, self.w, 3], np.uint8)
        self.model.run_once(init_input)  # TODO: Change inference once function

    def yolact_handler(self, request, response):
        print(request["image"])
        byte_data = base64.b64decode(request['image']['data'])
        np_img = np.frombuffer(byte_data, np.uint8)
        src = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
        if src.shape[0] == self.h and src.shape[1] == self.w:
            cv2.imshow('input', src)
            cv2.waitKey(1)
            response["result"] = self.model.run_once(src)  # TODO: Change inference once function
            print(response["result"])
            return True
        else:
            return False

    def run(self):
        # start receiving client call...
        service = roslibpy.Service(self.ros, self.srv_name, self.srv_type)
        service.advertise(self.yolact_handler)
        print("Service start!")
        self.ros.run_forever()
        # if roslibpy lose connect with ros master and reconnect,
        # topics will not re-subscribe, see:
        # https://github.com/gramaziokohler/roslibpy/issues/29


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='YOLACT ROS SERVER')
    parser.add_argument("--host", type=str, default="219.216.101.117", help="host of ros master")
    parser.add_argument("--port", type=int, default=9090, help="port of rosbridge")
    parser.add_argument("--srv_name", type=str, default="/run_yolact", help="advertise ros service name")
    parser.add_argument("--img_height", type=int, default=480, help="input image height")
    parser.add_argument("--img_width", type=int, default=640, help="input image width")
    parser.add_argument("--srv_type", type=str, default="semantic_msgs/RunInstance", help="advertise ros service type")
    parser.add_argument("--model_pth", type=str, default="./weights/yolact_base_54_800000.pth", help="weight path of yolact")
    parser.add_argument("--cuda_device", type=int, default=1, help="cude device id")
    args = parser.parse_args()

    os.environ["CUDA_VISIBLE_DEVICES"] = str(args.cuda_device)
    ys = YolactService(args.model_pth, args.srv_name, args.srv_type, args.host, args.port, args.img_height, args.img_width)
    ys.run()
