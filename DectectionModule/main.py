#!/usr/bin/env python3

import sys
import argparse
from constant import *

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput, Log


class DectectionModule:
    def __init__(self, input_source="/dev/video0"):
        # create video sources and outputs
        self.input = videoSource(input_source, argv=sys.argv)
        # load the object detection network
        self.net = None
        self.near_label = {}

    def StartEngine(self):
        self.net = detectNet(DEFAULT_MODEL, sys.argv, DETECT_THRESHOLD)

    def RunEngine(self):
        self.StartEngine()

        while True:
            # capture the next image
            img = input.Capture()

            if img is None:  # timeout
                continue

            # detect objects in the image (with overlay is label of object and confidence)
            detections = self.net.Detect(img, overlay="labels,conf")

            # print the detections
            print("detected {:d} objects in image".format(len(detections)))

            for detection in detections:
                print(detection)

            # # update the title bar
            # output.SetStatus(
            #     "{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS())
            # )

            # print out performance info
            self.net.PrintProfilerTimes()

            # exit on input/output EOS
            if not input.IsStreaming():
                break
