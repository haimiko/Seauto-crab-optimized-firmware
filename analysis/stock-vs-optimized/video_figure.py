#!/usr/bin/env python3
"""Run 9 against run 8 on the same floor, one picture. Reads video/study.json and video/same_floor.npz.

    python -I analysis/stock-vs-optimized/video_figure.py
"""
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3de"
OPTIMIZED, STOCK = "#2a78d6", "#eb6834"         # categorical slots 1 and 2, validated: dE 24.7 at worst for colour blindness
OPTIMIZED_LATE, STOCK_LATE = "#b7d3f6", "#f8c9b4"
NEVER = "#8d8c86"
BY_MINUTE = 30
FACTOR = 2                                      # drawn large, then shrunk, for smooth lines


def font(size, bold=False):
    return ImageFont.truetype(r"C:\Windows\Fonts\segoeuib.ttf" if bold else r"C:\Windows\Fonts\segoeui.ttf", size * FACTOR)


def rgb(colour):
    return tuple(int(colour[i:i + 2], 16) for i in (1, 3, 5))


def floor_map(floor, first, start_minute_cells, early, late):
    image = np.full(floor.shape + (3,), rgb(SURFACE), np.uint8)
    image[floor] = rgb(NEVER)
    image[floor & np.isfinite(first)] = rgb(late)
    image[floor & (first <= start_minute_cells)] = rgb(early)
    return image


def main():
    study = json.loads((HERE / "video/study.json").read_text())
    same = study["same_floor_by_assumed_depth"]["0.9"]
    data = np.load(HERE / "video/same_floor.npz")
    floor = data["floor"].astype(bool)
    ys, xs = np.nonzero(floor)
    box = (ys.min() - 4, ys.max() + 5, xs.min() - 4, xs.max() + 5)
    crop = lambda a: a[box[0]:box[1], box[2]:box[3]]

    width, height = 1560, 760
    canvas = Image.new("RGB", (width * FACTOR, height * FACTOR), rgb(SURFACE))
    draw = ImageDraw.Draw(canvas)

    def text(x, y, words, size=15, colour=INK, bold=False, anchor="la"):
        draw.text((x * FACTOR, y * FACTOR), words, fill=rgb(colour), font=font(size, bold), anchor=anchor)

    text(32, 24, f"Optimized cleans the same floor faster than stock, then both end close", 24, bold=True)
    text(32, 62, f"{same['floor_m2']} m\u00b2 that both cameras saw and at least one run reached. Time from the start of each cycle, to its battery floor.", 15, MUTED)

    # the maps
    scale = 4
    top, left = 112, 32
    panels = ((f"Optimized, run 8: {same['optimized_percent']['30']} % by minute {BY_MINUTE}", data["first8"], OPTIMIZED, OPTIMIZED_LATE, "later"),
              (f"Stock, run 9: {same['stock_percent']['30']} % by minute {BY_MINUTE}", data["first9"], STOCK, STOCK_LATE, "later"))
    map_h = (box[1] - box[0]) * scale
    map_w = (box[3] - box[2]) * scale
    for k, (title, first, early, late, _) in enumerate(panels):
        x = left + k * (map_w + 28)
        text(x, top, title, 16, bold=True)
        image = floor_map(crop(floor), crop(first), BY_MINUTE, early, late)
        image = cv2.resize(image, None, fx=scale * FACTOR, fy=scale * FACTOR, interpolation=cv2.INTER_NEAREST)
        canvas.paste(Image.fromarray(image), (x * FACTOR, (top + 30) * FACTOR))
        y = top + 30 + map_h + 12
        keys = ((early, f"cleaned by minute {BY_MINUTE}"), (late, "cleaned later"), (NEVER, "never, before the battery floor"))
        for colour, label in keys:
            draw.rectangle([x * FACTOR, y * FACTOR, (x + 14) * FACTOR, (y + 14) * FACTOR], fill=rgb(colour))
            text(x + 20, y - 3, label, 13, MUTED)
            y += 22

    # the curves
    plot_left, plot_right, plot_top, plot_bottom = left + 2 * map_w + 28 + 70, width - 112, top + 30, 640
    px = lambda minute: plot_left + (plot_right - plot_left) * minute / 90
    py = lambda percent: plot_bottom - (plot_bottom - plot_top) * percent / 100
    text(plot_left - 40, top, "Share of that floor cleaned (%)", 16, bold=True)
    for percent in range(0, 101, 25):
        draw.line([px(0) * FACTOR, py(percent) * FACTOR, px(90) * FACTOR, py(percent) * FACTOR], fill=rgb(GRID), width=FACTOR)
        text(plot_left - 10, py(percent) - 10, str(percent), 13, MUTED, anchor="ra")
    for minute in range(0, 91, 15):
        text(px(minute), plot_bottom + 8, str(minute), 13, MUTED, anchor="ma")
    text((plot_left + plot_right) / 2, plot_bottom + 30, "minutes into the cycle", 13, MUTED, anchor="ma")
    for key, colour in (("optimized_percent", OPTIMIZED), ("stock_percent", STOCK)):
        series = sorted((int(m), p) for m, p in same[key].items())
        points = [(px(0) * FACTOR, py(0) * FACTOR)] + [(px(m) * FACTOR, py(p) * FACTOR) for m, p in series]
        draw.line(points, fill=rgb(SURFACE), width=6 * FACTOR, joint="curve")
        draw.line(points, fill=rgb(colour), width=2 * FACTOR, joint="curve")
        for m, p in series:
            r = 4 * FACTOR
            draw.ellipse([px(m) * FACTOR - r - FACTOR * 2, py(p) * FACTOR - r - FACTOR * 2, px(m) * FACTOR + r + FACTOR * 2, py(p) * FACTOR + r + FACTOR * 2], fill=rgb(SURFACE))
            draw.ellipse([px(m) * FACTOR - r, py(p) * FACTOR - r, px(m) * FACTOR + r, py(p) * FACTOR + r], fill=rgb(colour))
    o, s = same["optimized_percent"], same["stock_percent"]
    text(px(30), py(o["30"]) - 28, f"{o['30']:.0f}", 14, bold=True, anchor="ma")
    text(px(30), py(s["30"]) + 10, f"{s['30']:.0f}", 14, bold=True, anchor="ma")
    text(px(88) + 12, py(o["88"]) - 22, f"Optimized {o['88']:.0f}", 14, bold=True)
    text(px(88) + 12, py(s["88"]) + 2, f"Stock {s['88']:.0f}", 14, bold=True)
    # the legend, as well as the direct labels
    for k, (colour, label) in enumerate(((OPTIMIZED, "Optimized, run 8"), (STOCK, "Stock, run 9"))):
        x = plot_left + 40 + k * 170
        y = plot_bottom + 62
        draw.line([x * FACTOR, (y + 9) * FACTOR, (x + 22) * FACTOR, (y + 9) * FACTOR], fill=rgb(colour), width=2 * FACTOR)
        draw.ellipse([(x + 7) * FACTOR, (y + 5) * FACTOR, (x + 15) * FACTOR, (y + 13) * FACTOR], fill=rgb(colour))
        text(x + 28, y - 1, label, 13, MUTED)
    seen8, seen9 = study["run8_seen_percent_to_floor"], study["seen_percent_to_floor"]
    text(32, height - 32, f"The camera followed the robot {seen8:.0f} % of run 8 and {seen9:.0f} % of run 9, so run 8's line is a lower bound. One run of each. The stock run also spent time on floor run 8's camera could not see.", 13, MUTED)
    canvas.resize((width, height), Image.LANCZOS).save(HERE / "video/same_floor.png")


if __name__ == "__main__":
    main()
