"""Generate and recognize OpenCV ArUco markers as arrays and SVG files."""

from __future__ import annotations

from pathlib import Path
from typing import Union
import re
import xml.etree.ElementTree as ET

import cv2
import numpy as np


ArrayLike = Union[np.ndarray, list[list[int]]]
MarkerSource = Union[str, Path, ArrayLike]


class ArucoMarker:
    """An ArUco marker with a configurable dictionary, ID, and physical size.

    ``as_array()`` returns a module-level array where 1 means black and 0
    means white. This convention makes the array directly useful as an
    OpenSCAD cut mask.
    """

    DEFAULT_DICTIONARY = "DICT_4X4_50"

    def __init__(
        self,
        marker_id: int,
        size_mm: float = 32.0,
        dictionary: str = DEFAULT_DICTIONARY,
        border_bits: int = 1,
    ) -> None:
        if size_mm <= 0:
            raise ValueError("size_mm must be greater than zero")
        if border_bits < 1:
            raise ValueError("border_bits must be at least 1")
        if not hasattr(cv2.aruco, dictionary):
            raise ValueError(f"Unknown OpenCV ArUco dictionary: {dictionary}")

        self.marker_id = int(marker_id)
        self.size_mm = float(size_mm)
        self.dictionary_name = dictionary
        self.border_bits = int(border_bits)
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary)
        )

        marker_count = int(self.dictionary.bytesList.shape[0])
        if not 0 <= self.marker_id < marker_count:
            raise ValueError(
                f"marker_id must be between 0 and {marker_count - 1} "
                f"for {dictionary}"
            )

    @property
    def payload_modules(self) -> int:
        return int(self.dictionary.markerSize)

    @property
    def total_modules(self) -> int:
        return self.payload_modules + 2 * self.border_bits

    def as_array(self) -> np.ndarray:
        """Return the marker modules as uint8 values: 1=black, 0=white."""
        image = cv2.aruco.generateImageMarker(
            self.dictionary,
            self.marker_id,
            self.total_modules,
            borderBits=self.border_bits,
        )
        return (image == 0).astype(np.uint8)

    def as_image(
        self, pixels_per_module: int = 100, quiet_zone_modules: int = 1
    ) -> np.ndarray:
        """Return a scan-ready grayscale image, including a white quiet zone."""
        if pixels_per_module < 1:
            raise ValueError("pixels_per_module must be at least 1")
        if quiet_zone_modules < 0:
            raise ValueError("quiet_zone_modules cannot be negative")

        modules = self.as_array()
        grayscale = np.where(modules == 1, 0, 255).astype(np.uint8)
        image = cv2.resize(
            grayscale,
            None,
            fx=pixels_per_module,
            fy=pixels_per_module,
            interpolation=cv2.INTER_NEAREST,
        )
        padding = quiet_zone_modules * pixels_per_module
        return cv2.copyMakeBorder(
            image,
            padding,
            padding,
            padding,
            padding,
            cv2.BORDER_CONSTANT,
            value=255,
        )

    def to_svg(self, path: str | Path) -> Path:
        """Write a millimetre-sized, vector-only SVG and return its path."""
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        modules = self.as_array()
        cell_mm = self.size_mm / self.total_modules

        root = ET.Element(
            "svg",
            {
                "xmlns": "http://www.w3.org/2000/svg",
                "width": f"{self.size_mm:g}mm",
                "height": f"{self.size_mm:g}mm",
                "viewBox": f"0 0 {self.size_mm:g} {self.size_mm:g}",
                "shape-rendering": "crispEdges",
                "data-dictionary": self.dictionary_name,
                "data-marker-id": str(self.marker_id),
                "data-modules": str(self.total_modules),
            },
        )
        for row, col in np.argwhere(modules == 1):
            ET.SubElement(
                root,
                "rect",
                {
                    "x": f"{col * cell_mm:.12g}",
                    "y": f"{row * cell_mm:.12g}",
                    "width": f"{cell_mm:.12g}",
                    "height": f"{cell_mm:.12g}",
                    "fill": "black",
                },
            )

        ET.indent(root, space="  ")
        ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
        return output

    def to_openscad_array(self) -> str:
        """Return the marker as an OpenSCAD-compatible nested array literal."""
        rows = (
            "[" + ",".join(str(int(value)) for value in row) + "]"
            for row in self.as_array()
        )
        return "[" + ",".join(rows) + "]"

    def recognize_id(self, source: MarkerSource) -> int | None:
        """Visually decode a marker ID from an SVG path or NumPy-like array.

        SVG input supports axis-aligned rectangle-based marker SVGs, including
        SVGs emitted by this class. The detector does not use filenames or SVG
        metadata to decide the ID.
        """
        if isinstance(source, (str, Path)):
            source_path = Path(source)
            if source_path.suffix.lower() == ".svg":
                image = self._svg_to_image(source_path)
            else:
                image = cv2.imread(str(source_path), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise ValueError(f"Could not read marker image: {source_path}")
                image = self._prepare_detection_image(image)
        else:
            image = self._prepare_detection_image(np.asarray(source))

        detector = cv2.aruco.ArucoDetector(
            self.dictionary, cv2.aruco.DetectorParameters()
        )
        _corners, ids, _rejected = detector.detectMarkers(image)
        if ids is None or len(ids) == 0:
            return None
        return int(np.asarray(ids).reshape(-1)[0])

    def _prepare_detection_image(self, array: np.ndarray) -> np.ndarray:
        if array.ndim == 3:
            array = cv2.cvtColor(array.astype(np.uint8), cv2.COLOR_BGR2GRAY)
        if array.ndim != 2:
            raise ValueError("Marker arrays must be two-dimensional")

        if array.size == 0:
            raise ValueError("Marker arrays cannot be empty")

        # Module masks returned by as_array() use 1=black. Ordinary images
        # conventionally use 0=black and 255=white.
        if float(np.max(array)) <= 1.0:
            grayscale = np.where(array > 0, 0, 255).astype(np.uint8)
        else:
            grayscale = np.clip(array, 0, 255).astype(np.uint8)

        target_size = max(grayscale.shape) * 100 if max(grayscale.shape) < 32 else max(grayscale.shape)
        image = cv2.resize(
            grayscale,
            (target_size, target_size),
            interpolation=cv2.INTER_NEAREST,
        )
        padding = max(20, target_size // max(grayscale.shape))
        return cv2.copyMakeBorder(
            image,
            padding,
            padding,
            padding,
            padding,
            cv2.BORDER_CONSTANT,
            value=255,
        )

    def _svg_to_image(self, path: Path) -> np.ndarray:
        root = ET.parse(path).getroot()
        view_box = root.get("viewBox")
        if not view_box:
            raise ValueError(f"SVG has no viewBox: {path}")
        values = [float(value) for value in re.split(r"[ ,]+", view_box.strip())]
        if len(values) != 4:
            raise ValueError(f"Invalid SVG viewBox: {view_box}")
        min_x, min_y, width, height = values

        modules = int(root.get("data-modules", self.total_modules))
        if modules < 1:
            raise ValueError("SVG module count must be positive")

        # Sample the centre of every expected module, honoring SVG painter's
        # order. This handles both one-rect-per-black-cell SVGs and files that
        # draw a black square followed by white cutout rectangles.
        sampled = np.full((modules, modules), 255, dtype=np.uint8)
        x_samples = min_x + (np.arange(modules) + 0.5) * width / modules
        y_samples = min_y + (np.arange(modules) + 0.5) * height / modules
        found_colored_rect = False

        for element in root.iter():
            if element.tag.rsplit("}", 1)[-1] != "rect":
                continue
            fill = self._parse_fill(element)
            if fill is None:
                continue
            found_colored_rect = True
            x = self._number(element.get("x", "0"))
            y = self._number(element.get("y", "0"))
            rect_width = self._number(element.get("width", "0"))
            rect_height = self._number(element.get("height", "0"))
            inside_x = (x_samples >= x) & (x_samples < x + rect_width)
            inside_y = (y_samples >= y) & (y_samples < y + rect_height)
            sampled[np.ix_(inside_y, inside_x)] = fill

        if not found_colored_rect:
            raise ValueError(
                "SVG recognition currently requires black/white <rect> elements"
            )
        return self._prepare_detection_image(sampled)

    @staticmethod
    def _number(value: str) -> float:
        match = re.match(r"\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", value)
        if not match:
            raise ValueError(f"Could not parse SVG number: {value!r}")
        return float(match.group(1))

    @staticmethod
    def _parse_fill(element: ET.Element) -> int | None:
        fill = element.get("fill", "")
        style = element.get("style", "")
        if not fill and style:
            match = re.search(r"(?:^|;)\s*fill\s*:\s*([^;]+)", style)
            fill = match.group(1).strip() if match else ""
        normalized = fill.strip().lower().replace(" ", "")
        if normalized in {"black", "#000", "#000000", "rgb(0,0,0)"}:
            return 0
        if normalized in {"white", "#fff", "#ffffff", "rgb(255,255,255)"}:
            return 255
        return None
