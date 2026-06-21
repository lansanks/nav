#!/usr/bin/env python3
"""Plot a top-down preview of the task field geometry YAML."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import yaml


def _center_rect(ax, center, size, *, facecolor, edgecolor, label, alpha=0.45, zorder=2):
    x = float(center["x"])
    y = float(center["y"])
    width_x = float(size["width_x"])
    length_y = float(size["length_y"])
    rect = patches.Rectangle(
        (x - width_x / 2.0, y - length_y / 2.0),
        width_x,
        length_y,
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=1.5,
        alpha=alpha,
        zorder=zorder,
    )
    ax.add_patch(rect)
    ax.text(x, y, label, ha="center", va="center", fontsize=8, zorder=zorder + 1)
    return rect


def _draw_wall(ax, wall):
    typ = wall.get("type")
    if typ == "line_segment":
        start = wall["start"]
        end = wall["end"]
        ax.plot(
            [start["x"], end["x"]],
            [start["y"], end["y"]],
            color="black",
            linewidth=3.0,
            solid_capstyle="butt",
            zorder=5,
        )
        mid_x = (start["x"] + end["x"]) / 2.0
        mid_y = (start["y"] + end["y"]) / 2.0
        ax.text(mid_x, mid_y, wall["id"], fontsize=7, color="black", ha="center", va="center")
    elif typ == "gap":
        start = wall["start"]
        end = wall["end"]
        ax.plot(
            [start["x"], end["x"]],
            [start["y"], end["y"]],
            color="white",
            linewidth=5.0,
            solid_capstyle="butt",
            zorder=6,
        )
        ax.text(
            (start["x"] + end["x"]) / 2.0,
            (start["y"] + end["y"]) / 2.0 - 0.12,
            "entrance",
            fontsize=8,
            color="black",
            ha="center",
            va="top",
        )


def plot_map(yaml_path: Path, output_path: Path, show: bool = False) -> None:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    geom = data["map_geometry"]

    fig, ax = plt.subplots(figsize=(8, 9))
    ax.set_title(f"{geom['name']} ({geom['frame']['frame_id']})")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.4, alpha=0.45)

    boundary = geom["field"]["boundary"]
    field_x = boundary["min_x"]
    field_y = boundary["min_y"]
    field_w = boundary["max_x"] - boundary["min_x"]
    field_h = boundary["max_y"] - boundary["min_y"]
    ax.add_patch(
        patches.Rectangle(
            (field_x, field_y),
            field_w,
            field_h,
            facecolor="#ffff66",
            edgecolor="black",
            linewidth=1.5,
            alpha=0.55,
            zorder=0,
        )
    )
    ax.text(
        boundary["min_x"] + 0.05,
        boundary["max_y"] - 0.10,
        "field 4m x 6m",
        ha="left",
        va="top",
        fontsize=9,
        zorder=7,
    )

    for wall in geom.get("walls", []):
        _draw_wall(ax, wall)

    start = geom["start_area"]
    _center_rect(
        ax,
        start["center"],
        start["size"],
        facecolor="#ffaaaa",
        edgecolor="#aa0000",
        label="start",
        alpha=0.5,
        zorder=3,
    )

    bump = geom["speed_bump"]
    _center_rect(
        ax,
        bump["center"],
        bump["size"],
        facecolor="#df7500",
        edgecolor="#aa5500",
        label="speed bump",
        alpha=0.75,
        zorder=2,
    )

    for slot in geom["storage_slots"]["slots"]:
        _center_rect(
            ax,
            slot["center"],
            slot["size"],
            facecolor="#8c6508",
            edgecolor="#5e4404",
            label=f"S{slot['id']}",
            alpha=0.75,
            zorder=4,
        )

    zone_colors = {
        "food": "#75bd42",
        "tool": "#808080",
        "instrument": "#2e54a1",
        "medicine": "#c81d31",
    }
    for zone in geom["return_zones"]["zones"]:
        _center_rect(
            ax,
            zone["center"],
            zone["size"],
            facecolor=zone_colors.get(zone["name"], "#dddddd"),
            edgecolor="black",
            label=f"R{zone['id']}\n{zone['name']}",
            alpha=0.8,
            zorder=4,
        )

    q = geom.get("question_display_area")
    if q and q.get("center"):
        ax.scatter([q["center"]["x"]], [q["center"]["y"]], marker="s", s=80, color="white", edgecolor="black", zorder=6)
        ax.text(q["center"]["x"], q["center"]["y"] + 0.10, "display", ha="center", va="bottom", fontsize=8, zorder=7)

    margin = 0.75
    ax.set_xlim(boundary["min_x"] - margin, boundary["max_x"] + margin)
    ax.set_ylim(boundary["min_y"] - margin, boundary["max_y"] + margin)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    if show:
        plt.show()
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--yaml",
        default="/home/linkchen/black_mujoco/src/navigation/config/mission/map_geom.yaml",
        help="Path to map geometry YAML.",
    )
    parser.add_argument(
        "--output",
        default="/home/linkchen/black_mujoco/src/navigation/config/mission/map_geom_preview.png",
        help="Output preview image path.",
    )
    parser.add_argument("--show", action="store_true", help="Open an interactive matplotlib window.")
    args = parser.parse_args()

    plot_map(Path(args.yaml), Path(args.output), show=args.show)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
