"""The ChArUco calibration sheet (docs/13 §4.5): a chessboard with ArUco markers in its white squares. Every inner
corner has a known position on the sheet, and the markers say which corner is which even when only part of the
sheet is in the picture.

Marker ids start at 100 of DICT_4X4_250, so they never look like a bed label (DICT_4X4_50, ids 0–49: the first 50
markers of DICT_4X4_250 are the same as DICT_4X4_50).
Print the PDF at 100 % ("actual size") on a laser printer, matte paper; both check bars must measure 100.0 mm.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

SQUARES = (12, 9)            # columns, rows
SQUARE_MM = 5.0
MARKER_MM = 3.6
DICTIONARY = cv2.aruco.DICT_4X4_250
FIRST_ID = 100


def board(squares=SQUARES, square_mm: float = SQUARE_MM, marker_mm: float = MARKER_MM, first_id: int = FIRST_ID):
    d = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    n = (squares[0] * squares[1]) // 2
    return cv2.aruco.CharucoBoard(squares, square_mm, marker_mm, d, np.arange(first_id, first_id + n, dtype=np.int32))


@dataclass
class Detection:
    ids: np.ndarray            # corner ids (N)
    px: np.ndarray             # corner pixels (N x 2)
    sheet_mm: np.ndarray       # corner positions on the sheet (N x 2), mm, Y up (row 0 at the bottom)


class Detector:
    def __init__(self, b=None) -> None:
        self.board = b or board()
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.det = cv2.aruco.CharucoDetector(self.board, cv2.aruco.CharucoParameters(), params)
        pts = np.asarray(self.board.getChessboardCorners(), np.float64)[:, :2]
        h = self.board.getChessboardSize()[1] * self.board.getSquareLength()
        self.corners_mm = np.c_[pts[:, 0], h - pts[:, 1]]     # OpenCV's sheet Y points down: make it Y up

    def find(self, gray: np.ndarray, min_corners: int = 6) -> Detection | None:
        img = gray if gray.ndim == 2 else cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        corners, ids, _, _ = self.det.detectBoard(img)
        if ids is None or len(ids) < min_corners:
            return None
        ids = ids.ravel().astype(int)
        return Detection(ids, corners.reshape(-1, 2).astype(np.float64), self.corners_mm[ids])


def sheet_image(px_per_mm: float, b=None, margin_mm: float = 4.0) -> np.ndarray:
    """The sheet as a grey image (white margin around it), px_per_mm pixels per mm."""
    b = b or board()
    cols, rows = b.getChessboardSize()
    w_mm, h_mm = cols * b.getSquareLength(), rows * b.getSquareLength()
    img = b.generateImage((int(round(w_mm * px_per_mm)), int(round(h_mm * px_per_mm))), marginSize=0, borderBits=1)
    m = int(round(margin_mm * px_per_mm))
    return cv2.copyMakeBorder(img, m, m, m, m, cv2.BORDER_CONSTANT, value=255)


def sheet_size_mm(b=None) -> tuple[float, float]:
    b = b or board()
    cols, rows = b.getChessboardSize()
    return cols * b.getSquareLength(), rows * b.getSquareLength()


def write_pdf(path: Path, dpi: int = 600) -> Path:
    """An A4 page: two copies of the sheet (one for the bed, one for the block) and two 100 mm check bars."""
    from PIL import Image, ImageDraw
    ppm = dpi / 25.4
    page = Image.new("L", (int(210 * ppm), int(297 * ppm)), 255)
    sheet = Image.fromarray(sheet_image(ppm))
    page.paste(sheet, (int(20 * ppm), int(20 * ppm)))
    page.paste(sheet, (int(20 * ppm), int(20 * ppm) + sheet.height + int(15 * ppm)))
    d = ImageDraw.Draw(page)
    y = int(230 * ppm)
    d.rectangle([int(20 * ppm), y, int(120 * ppm), y + int(2 * ppm)], fill=0)            # 100 mm across
    x = int(180 * ppm)
    d.rectangle([x, int(150 * ppm), x + int(2 * ppm), int(250 * ppm)], fill=0)           # 100 mm down
    d.text((int(20 * ppm), y + int(4 * ppm)), "100 mm: measure both bars. Print at 100 % (actual size).", fill=0)
    path.parent.mkdir(parents=True, exist_ok=True)
    page.save(path, "PDF", resolution=dpi)
    return path
