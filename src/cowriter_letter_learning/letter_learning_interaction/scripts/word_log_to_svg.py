#!/usr/bin/env python3
# flake8: noqa
"""
word_log_to_svg.py

This script reads a log file and extracts the path information to
generate SVG code, which is then printed to standard output.

To run the script, execute the following command:

    python3 word_log_to_svg.py /path/to/log_file

Note: 
    - The log file must contain path information in the format
      "[[x1, y1], [x2, y2], ...]".
    - The SVG code will be printed to standard output.
"""


from ast import literal_eval
import sys
import re

# Regular expression to extract the path from the log line
action = re.compile(".*INFO - (?P<path>[.*\])")

# Set the scale factor
SCALE: int = 2000

if __name__ == "__main__":
    # Print the SVG header
    print(
        """<?xml version="1.0" standalone="no"?>
<svg width="210mm" height="297mm" version="1.1" xmlns="http://www.w3.org/2000/svg">
"""
    )

    with open(sys.argv[1], "r") as log:
        for line in log.readlines():
            # Search for a line containing the path information
            found = action.search(line)
            if found:
                # Extract the path from the line
                paths = literal_eval(found.group("path"))
                print("\t<g>")
                for path in paths:
                    # Write the SVG path element with the scaled coordinates
                    sys.stdout.write(
                        f'\t\t<path d="M{path[0][0] * SCALE} {path[0][1] * SCALE} '
                    )
                    for x, y in path:
                        x *= SCALE
                        y *= SCALE
                        sys.stdout.write(f"L{x} {y} ")
                    sys.stdout.write(
                        '" style="fill:none;stroke:#000000;stroke-width:2.0"/>'
                    )
                print("\t</g>")

    # Print the SVG footer
    print("</svg>")
