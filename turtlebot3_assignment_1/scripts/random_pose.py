#!/usr/bin/env python3
from pysdf import SDF
import numpy as np
import random
from ament_index_python.packages import get_package_share_directory
import os
import argparse


class SelectRandomPose:

    def __init__(self, sdl_path, min_x=-1.5, min_y=-1.5, max_x=2, max_y=2):

        self.parsed_sdl = SDF.from_file(sdl_path, remove_blank_text=True)
        self.parsed_sdl.to_file(sdl_path, pretty_print=True)
        self.objects_to_ignore = [
            "head",
            "left_hand",
            "right_hand",
            "left_foot",
            "right_foot",
            "body",
        ]
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y

        self.x = np.linspace(min_x, max_x, 100)
        self.y = np.linspace(min_y, max_y, 100)

    def get_random_pose(self):
        self.cylinder_poses = []
        for visual in self.parsed_sdl.iter("visual"):
            if visual.name not in self.objects_to_ignore:
                w, h, d = visual.geometry.box.size
                if w == h == d == 1:
                    r = visual.geometry.cylinder.radius
                    w = h = float(r) * 2

                pose = np.fromstring(visual.pose.text, count=6, sep=" ")
                x, y = pose[0], pose[1]

                x_min = x - (w / 2) - 0.05
                x_max = x + (w / 2) + 0.05

                self.x[(self.x > x_min) & (self.x < x_max)] = -1000

                y_min = y - (h / 2) - 0.05
                y_max = y + (h / 2) + 0.05

                self.y[(self.y > y_min) & (self.y < y_max)] = -1000

        x_c = -1000
        y_c = -1000
        while x_c == -1000 or y_c == -1000:
            if x_c == -1000:
                x_c = random.choice(self.x)
            if y_c == -1000:
                y_c = random.choice(self.y)
        return x_c, y_c


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Get random pose")
    parser.add_argument(
        "-g",
        required=True,
        type=str,
        help="Group name",
    )
    parser.add_argument(
        "-w",
        required=True,
        type=str,
        help="Group world",
    )
    args = parser.parse_args()

    dir = get_package_share_directory("turtlebot3_assignment_1")
    path = os.path.join(
        dir,
        "models",
        f"g{args.g}w{args.w}",
        "model.sdf",
    )
    random_pose = SelectRandomPose(path)
    pose = random_pose.get_random_pose()
    print(f"x_pose:={pose[0]}", f"y_pose:={pose[1]}")
