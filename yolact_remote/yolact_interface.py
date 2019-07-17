from yolact import Yolact
from utils.augmentations import FastBaseTransform
from layers.output_utils import postprocess
import pycocotools
from data import cfg, set_cfg

import numpy as np
import torch
import torch.backends.cudnn as cudnn
import time
import json
import os

import cv2


class YolactInterface(object):
    def __init__(self, model_pth, output_num=5):
        self.output_num = output_num
        with torch.no_grad():
            set_cfg("yolact_base_config")
            torch.cuda.set_device(0)
            cudnn.benchmark = True
            cudnn.fastest = True
            torch.set_default_tensor_type('torch.cuda.FloatTensor')
            self.net = Yolact()
            self.net.load_weights(model_pth)
            self.net.eval()
            self.net = self.net.cuda()
        print("load model complete")

    def run_once(self, src):
        self.net.detect.cross_class_nms = True
        self.net.detect.use_fast_nms = True
        cfg.mask_proto_debug = False
        with torch.no_grad():
            frame = torch.Tensor(src).cuda().float()
            batch = FastBaseTransform()(frame.unsqueeze(0))
            time_start = time.clock()
            preds = self.net(batch)
            time_elapsed = (time.clock() - time_start)
            h, w, _ = src.shape
            t = postprocess(preds, w, h, visualize_lincomb=False, crop_masks=True,
                            score_threshold=0.)  # TODO: give a suitable threshold
            torch.cuda.synchronize()
            classes, scores, bboxes, masks = [x[:self.output_num].cpu().numpy() for x in
                                             t]  # TODO: Only 5 objects for test
            print(time_elapsed)
        instances = self.build_up_result(masks.shape[0], classes, bboxes, masks, scores)
        return {"instances": instances}

    def build_up_result(self, num, classes, bboxes, masks, scores):
        instances = []
        for i in range(num):
            bbox = [bboxes[i, 0], bboxes[i, 1], bboxes[i, 2] - bboxes[i, 0], bboxes[i, 3] - bboxes[i, 1]]
            # Round to the nearest 10th to avoid huge file sizes, as COCO suggests
            bbox = [round(float(x) * 10) / 10 for x in bbox]
            # encode segmentation with RLE
            rle = pycocotools.mask.encode(
                np.asfortranarray(masks[i, :, :].astype(np.uint8)))  # rle binary encoding
            rle['counts'] = rle['counts'].decode('ascii')  # json.dump doesn't like bytes strings
            # create one instance json
            instances.append({
                    'category_id': int(classes[i]),  # TODO: origin: get_coco_cat(int(category_id))
                    'bbox': {"b": bbox},
                    "segmentation": rle,
                    'score': float(scores[i])
                })

        return instances
