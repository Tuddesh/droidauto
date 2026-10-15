#!/usr/bin/env python3
"""Interactively inspect YCB-V BOP test images and ground-truth classes.

Run from the droidauto project root:
    conda run -n droidauto python maskrcnn_experiments/browse_ycbv.py

Controls: Right/Down/N = next image, Left/Up/P = previous image,
Home/End = first/last image, Q/Escape = quit.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from PIL import Image


YCBV_CLASSES = {
    1: "master_chef_can", 2: "cracker_box", 3: "sugar_box", 4: "tomato_soup_can",
    5: "mustard_bottle", 6: "tuna_fish_can", 7: "pudding_box", 8: "gelatin_box",
    9: "potted_meat_can", 10: "banana", 11: "pitcher_base", 12: "bleach_cleanser",
    13: "bowl", 14: "mug", 15: "power_drill", 16: "wood_block", 17: "scissors",
    18: "large_marker", 19: "large_clamp", 20: "extra_large_clamp", 21: "foam_brick",
}


def parse_args() -> argparse.Namespace:
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-root", type=Path, default=project_root / "data" / "ycbv_test_bop19" / "test",
        help="BOP test directory (default: the downloaded YCB-V subset)",
    )
    parser.add_argument("--scene", help="Show only one scene, e.g. 000048")
    parser.add_argument("--start", type=int, default=0, help="Zero-based initial image index")
    parser.add_argument(
        "--full-masks", action="store_true",
        help="Overlay full masks instead of the default visible-region masks",
    )
    return parser.parse_args()


def collect_frames(data_root: Path, scene_filter: str | None) -> list[tuple[Path, Path]]:
    """Return (scene directory, RGB path) pairs in stable scene/image order."""
    scenes = sorted(path for path in data_root.iterdir() if path.is_dir())
    if scene_filter:
        scenes = [path for path in scenes if path.name == scene_filter]
    frames = [(scene, rgb) for scene in scenes for rgb in sorted((scene / "rgb").glob("*.png"))]
    if not frames:
        raise FileNotFoundError(f"No RGB frames found in {data_root}. Check --data-root and --scene.")
    return frames


class YCBVBrowser:
    def __init__(self, frames: list[tuple[Path, Path]], start: int, use_full_masks: bool) -> None:
        self.frames = frames
        self.index = max(0, min(start, len(frames) - 1))
        self.mask_folder = "mask" if use_full_masks else "mask_visib"
        self.annotation_cache: dict[Path, tuple[dict, dict]] = {}
        self.figure, (self.image_axis, self.info_axis) = plt.subplots(
            1, 2, figsize=(15, 8), gridspec_kw={"width_ratios": [3.2, 1]}
        )
        self.figure.canvas.mpl_connect("key_press_event", self.on_key)
        self.render()

    def annotations_for(self, scene: Path) -> tuple[dict, dict]:
        if scene not in self.annotation_cache:
            with (scene / "scene_gt.json").open(encoding="utf-8") as handle:
                ground_truth = json.load(handle)
            with (scene / "scene_gt_info.json").open(encoding="utf-8") as handle:
                info = json.load(handle)
            self.annotation_cache[scene] = ground_truth, info
        return self.annotation_cache[scene]

    def render(self) -> None:
        scene, rgb_path = self.frames[self.index]
        image_id = rgb_path.stem
        ground_truth, info = self.annotations_for(scene)
        instances = ground_truth[str(int(image_id))]
        instance_info = info[str(int(image_id))]

        self.image_axis.clear()
        self.info_axis.clear()
        self.image_axis.imshow(np.asarray(Image.open(rgb_path).convert("RGB")))
        self.image_axis.set_axis_off()
        self.image_axis.set_title(f"Scene {scene.name} · image {image_id} · {self.index + 1}/{len(self.frames)}")

        lines = ["Ground-truth classes", ""]
        cmap = plt.get_cmap("tab20")
        for instance_index, (annotation, annotation_info) in enumerate(zip(instances, instance_info)):
            object_id = annotation["obj_id"]
            name = YCBV_CLASSES.get(object_id, f"unknown_class_{object_id}")
            colour = cmap(instance_index % 20)
            mask_path = scene / self.mask_folder / f"{image_id}_{instance_index:06d}.png"
            if mask_path.exists():
                mask = np.asarray(Image.open(mask_path)) > 0
                overlay = np.zeros((*mask.shape, 4), dtype=float)
                overlay[..., :3] = colour[:3]
                overlay[..., 3] = mask * 0.35
                self.image_axis.imshow(overlay)

            x, y, width, height = annotation_info["bbox_visib"]
            self.image_axis.add_patch(Rectangle((x, y), width, height, fill=False, edgecolor=colour, linewidth=2))
            self.image_axis.text(
                x, max(y - 4, 10), f"{object_id}: {name}", color="white", fontsize=9,
                bbox={"facecolor": colour, "edgecolor": "none", "alpha": 0.9, "pad": 2},
            )
            lines.extend((f"{object_id:2d}  {name.replace('_', ' ')}", f"    visible: {annotation_info['visib_fract']:.1%}"))

        self.info_axis.set_axis_off()
        self.info_axis.text(0, 1, "\n".join(lines), va="top", fontsize=11, family="monospace")
        self.figure.suptitle(f"YCB-V labels · {self.mask_folder} overlay · ←/→ browse · Q quit", fontsize=13)
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()

    def on_key(self, event) -> None:
        if event.key in {"right", "down", "n"}:
            self.index = min(self.index + 1, len(self.frames) - 1)
        elif event.key in {"left", "up", "p"}:
            self.index = max(self.index - 1, 0)
        elif event.key == "home":
            self.index = 0
        elif event.key == "end":
            self.index = len(self.frames) - 1
        elif event.key in {"q", "escape"}:
            plt.close(self.figure)
            return
        else:
            return
        self.render()


def main() -> None:
    args = parse_args()
    frames = collect_frames(args.data_root, args.scene)
    YCBVBrowser(frames, args.start, args.full_masks)
    plt.show()


if __name__ == "__main__":
    main()
