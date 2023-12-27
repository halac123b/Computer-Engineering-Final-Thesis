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
        self.current_select = []

    def StartEngine(self):
        self.net = detectNet(DEFAULT_MODEL, sys.argv, DETECT_THRESHOLD)

    def RunEngine(self):
        # capture the next image
        img = input.Capture()

        if img is None:  # timeout
            return

        # detect objects in the image (with overlay is label of object and confidence)
        detections = self.net.Detect(img, overlay="labels,conf")

        # print the detections
        print("detected {:d} objects in image".format(len(detections)))

        for detection in detections:
            for key in self.near_label:
                self.near_label[key] -= 1
                if self.near_label[key] == 0:
                    del self.near_label[key]

            # print(detection)
            label = self.net.GetClassDesc(detection.ClassID)
            if label in self.near_label:
                self.near_label[label] += 1
            else:
                self.near_label[label] = 1

            self.current_select.clear()
            self.current_select.append(label)

        # # update the title bar
        # output.SetStatus(
        #     "{:s} | Network {:.0f} FPS".format(args.network, net.GetNetworkFPS())
        # )

        # print out performance info
        self.net.PrintProfilerTimes()

        # exit on input/output EOS
        if not input.IsStreaming():
            return

    def GetChosenLabel(self):
        for label in self.current_select:
            if self.near_label[label] <= 2:
                self.current_select.remove(label)
        return self.current_select
