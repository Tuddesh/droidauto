# Source record — YCB-V BOP'19–24 test subset

## Dataset identity

- **Dataset:** YCB-Video (YCB-V), in BOP format.
- **Subset:** `ycbv_test_bop19` — the BOP'19–24 selected test images.
- **Contents:** 900 RGB-D images across 12 scenes; RGB, depth, complete masks,
  visible masks, camera parameters, object IDs, visibility statistics, and 6D
  ground-truth poses.
- **Purpose in this project:** held-out evaluation of Mask R-CNN instance
  segmentation. It must not be used for training, model selection, threshold
  tuning, or augmentation fitting.

## Authoritative sources

1. BOP dataset catalogue: <https://bop.felk.cvut.cz/datasets/>
2. Downloaded distribution: <https://huggingface.co/datasets/bop-benchmark/ycbv>
3. Exact retrieved asset: <https://huggingface.co/datasets/bop-benchmark/ycbv/resolve/main/ycbv_test_bop19.zip>
4. Original YCB-Video / PoseCNN project: <https://rse-lab.cs.washington.edu/projects/posecnn/>
5. BOP format specification: <https://github.com/thodan/bop_toolkit/blob/master/docs/bop_datasets_format.md>

The BOP catalogue describes YCB-V as 21 YCB objects captured in 92 videos. For
the BOP evaluation subset, 75 images were selected from each of 12 test scenes
to reduce redundancy and exclude images with erroneous ground-truth poses.

## Licence and attribution

The BOP catalogue identifies YCB-V / the original PoseCNN project as MIT
licensed. Retain this source record when copying or redistributing this data,
and cite Xiang et al. (2018) and Hodaň et al. (2018), as listed in the project
README.

## Local acquisition record

- **Retrieved on:** 2026-09-28 (Europe/Prague)
- **Retrieved by:** `curl -fL --retry 3 --retry-delay 2`
- **Downloaded archive name:** `ycbv_test_bop19.zip`
- **Archive transfer size:** 629 MiB (advertised distribution: 660 MB)
- **Extracted size:** 669 MiB
- **Archive disposition:** removed after successful extraction to stay below
  the project's 2 GB data limit.
- **Extracted file count:** 10,086 files under `test/` (900 RGB images).

### Integrity spot-check

The following SHA-256 was calculated after extraction:

```text
45e6bbd9e7cd2403d45f4fbbe70944173f781fb5dcd8f9b51dad0840713f5c68  test/000048/scene_gt.json
```

To repeat it:

```bash
sha256sum data/ycbv_test_bop19/test/000048/scene_gt.json
```
