#!/usr/bin/env python3
"""Generate random, rule-valid AirMouse mazes as Gazebo (SDF) worlds from a random seed.

Planned for Phase 1:
  - a grid of 1 m cells inside at most 15 m x 15 m;
  - corridors carved with a randomized depth-first search, then a few extra walls removed
    so the maze has loops;
  - 2 m x 2 m rooms (target 6-8) reachable from corridors;
  - walls 2.44 m tall; one entry/exit opening on the outer wall;
  - up to 6 simple human-sized survivor models inside rooms;
  - output: an SDF world file, plus a ground-truth occupancy image + YAML of the layout;
  - asserts that every room is reachable and every corridor is at least 1 m wide.
"""

import argparse
import sys


def build_arg_parser() -> argparse.ArgumentParser:
    """Create the command-line parser for the maze generator.

    Returns:
        The configured argument parser.
    """
    return argparse.ArgumentParser(description=__doc__.splitlines()[0])


def main(argv: list[str] | None = None) -> int:
    """Entry point: parse arguments and generate a maze world.

    Args:
        argv: Command-line arguments (defaults to sys.argv[1:]).

    Returns:
        Process exit code.
    """
    build_arg_parser().parse_args(argv)
    sys.stderr.write("generate_maze.py: not implemented yet (Phase 1)\n")
    return 1


if __name__ == "__main__":
    sys.exit(main())
