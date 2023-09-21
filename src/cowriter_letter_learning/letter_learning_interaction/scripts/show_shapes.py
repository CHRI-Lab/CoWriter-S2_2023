#!/usr/bin/env python3

"""
display_shapes.py

This script displays shapes stored in a CSV file as matplotlib plots.

To run this script, pass the name of the CSV file containing the shapes
as an argument. Optionally, you can use the --no_clear flag to avoid 
clearing the display between shapes.

Usage:
    python3 display_shapes.py input_file.csv [--no_clear]

Args:
    input_file.csv: A string containing the name of CSV file to read
                    from.

Options:
    --no_clear: Don't clear the display (useful for viewing shapes in 
                proportion to each other).

Requirements:
    - Python 3.x
    - Matplotlib
"""

import matplotlib.pyplot as plt
import numpy as np
import argparse
import csv
import time
from typing import List


def show_shapes(input_filename: str, no_clear: bool) -> None:
    """
    Displays shapes stored in a CSV file as matplotlib plots.

    :param input_filename: A string containing the name of the CSV file
                           to read from.
    :param no_clear: A boolean flag indicating whether to clear the 
                     display between shapes.
    """
    plt.ion()

    with open(input_filename, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',', quotechar='|')
        for row in csv_reader:
            _stroke: List[str] = row[2:]
            stroke: List[float] = [float(coordinate) for coordinate in _stroke]
            # The x- and y-coordinates of the shape
            # Used to plot the shape
            x_shape, y_shape = stroke[::2], stroke[1::2]

            # If no_clear is false then clear plot before drawing next 
            # shape
            if not no_clear:
                plt.clf()

            plt.plot(np.asarray(x_shape), np.asarray(y_shape), linewidth=10)
            plt.draw()
            time.sleep(1.0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Show shapes in csv file')
    parser.add_argument(
        'input', 
        help='A string containing the name of the CSV file to read from.')
    parser.add_argument(
        '--no_clear', 
        action='store_true',
        help="Don't clear the display (useful for viewing shapes in" +
            " proportion to each other).")
    args = parser.parse_args()

    show_shapes(args.input, args.no_clear)
