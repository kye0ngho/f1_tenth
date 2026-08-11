#!/usr/bin/env python3
"""Plot the optimized raceline and closed-loop controller trajectories."""

import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


def load_xy(csv_path, x_name='x_m', y_name='y_m'):
    with open(csv_path, newline='') as csv_file:
        rows = list(csv.DictReader(csv_file))
    return np.asarray([
        (float(row[x_name]), float(row[y_name])) for row in rows
    ])


def parse_trajectory_arg(value):
    label, color, csv_path = value.split(':', 2)
    return label, color, csv_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-yaml', required=True)
    parser.add_argument('--raceline', required=True)
    parser.add_argument(
        '--trajectory', action='append', required=True,
        type=parse_trajectory_arg,
        help='LABEL:COLOR:CSV_PATH, repeatable (one per controller run)')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    with open(args.map_yaml) as yaml_file:
        map_config = yaml.safe_load(yaml_file)
    map_image_path = map_config['image']
    if not map_image_path.startswith('/'):
        import os
        map_image_path = os.path.join(
            os.path.dirname(args.map_yaml), map_image_path)
    image = np.asarray(Image.open(map_image_path))
    resolution = float(map_config['resolution'])
    origin_x, origin_y, _ = map_config['origin']
    height, width = image.shape[:2]
    extent = (
        origin_x,
        origin_x + width * resolution,
        origin_y,
        origin_y + height * resolution,
    )

    raceline = load_xy(args.raceline)

    figure, axis = plt.subplots(figsize=(8, 7), constrained_layout=True)
    axis.imshow(image, cmap='gray', origin='upper', extent=extent)
    axis.plot(raceline[:, 0], raceline[:, 1], color='#19b34a', linewidth=2.0,
              label='Optimized safe raceline')
    for label, color, csv_path in args.trajectory:
        trajectory = load_xy(csv_path, 'x_m', 'y_m')
        axis.plot(trajectory[::30, 0], trajectory[::30, 1],
                  color=color, linewidth=1.3, alpha=0.9, label=label)
    axis.set_aspect('equal', adjustable='box')
    axis.set_xlabel('map x [m]')
    axis.set_ylabel('map y [m]')
    axis.set_title('track02 closed-loop controller comparison')
    axis.legend(loc='best')
    figure.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == '__main__':
    main()
