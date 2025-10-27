#!/usr/bin/env python3
# Simple BEV plotter for cone YAML files that live in this folder.
# Automatically saves a PNG beside the YAML.
#
# Usage:
#   python3 plot_map.py                # plots fsg24.yaml (default)
#   python3 plot_map.py some_map.yaml  # plots another YAML
#
# Requires: PyYAML, matplotlib
#   pip install pyyaml matplotlib

import argparse
import pathlib
import sys
import yaml
import matplotlib.pyplot as plt


def load_yaml_cones(yaml_path: pathlib.Path):
    data = yaml.safe_load(yaml_path.read_text())
    cones = data.get("cones", [])
    xs = [float(c["x"]) for c in cones]
    ys = [float(c["y"]) for c in cones]
    title = yaml_path.stem
    return xs, ys, title


def main():
    here = pathlib.Path(__file__).parent
    p = argparse.ArgumentParser(description="Plot cone YAML and save PNG next to it.")
    p.add_argument(
        "yaml",
        nargs="?",
        default="fsg24.yaml",
        help="YAML filename in this folder (default: fsg24.yaml)",
    )
    args = p.parse_args()

    yaml_path = (here / args.yaml).resolve()
    if not yaml_path.exists():
        sys.exit(f"YAML not found: {yaml_path}")

    xs, ys, title = load_yaml_cones(yaml_path)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(xs, ys, s=18)
    ax.set_aspect("equal", adjustable="datalim")  # keeps x/y scale uniform
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    plt.tight_layout()

    png_path = yaml_path.with_suffix(".png")
    fig.savefig(png_path, dpi=180, bbox_inches="tight")
    print(f"Saved: {png_path}")
    plt.show()


if __name__ == "__main__":
    main()
