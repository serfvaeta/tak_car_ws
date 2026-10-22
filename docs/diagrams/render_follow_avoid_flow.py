"""Render the current FOLLOW_AVOID cycle to SVG and PNG."""

from html import escape
from math import atan2, cos, pi, sin
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
WIDTH, HEIGHT = 1200, 650
image = Image.new("RGB", (WIDTH, HEIGHT), "#f8fafc")
draw = ImageDraw.Draw(image)
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" height="{HEIGHT}">',
       '<rect width="100%" height="100%" fill="#f8fafc"/>']


def text(x, y, value, size=22):
    draw.text((x, y), value, font=ImageFont.truetype(FONT, size),
              fill="#172033", anchor="mm")
    svg.append(f'<text x="{x}" y="{y}" text-anchor="middle" '
               f'dominant-baseline="central" font-family="Noto Sans CJK SC" '
               f'font-size="{size}" fill="#172033">{escape(value)}</text>')


def card(x, y, title, detail, fill):
    draw.rounded_rectangle((x, y, x + 280, y + 120), 18,
                           fill=fill, outline="#64748b", width=2)
    svg.append(f'<rect x="{x}" y="{y}" width="280" height="120" rx="18" '
               f'fill="{fill}" stroke="#64748b" stroke-width="2"/>')
    text(x + 140, y + 38, title, 27)
    text(x + 140, y + 84, detail, 19)


def arrow(points):
    draw.line(points, fill="#475569", width=4)
    x0, y0 = points[-2]
    x1, y1 = points[-1]
    angle = atan2(y1 - y0, x1 - x0)
    head = [(x1, y1)] + [
        (x1 - 14 * cos(angle + turn), y1 - 14 * sin(angle + turn))
        for turn in (-pi / 6, pi / 6)
    ]
    draw.polygon(head, fill="#475569")
    svg.append('<polyline points="' +
               " ".join(f"{x},{y}" for x, y in points) +
               '" fill="none" stroke="#475569" stroke-width="4"/>')
    svg.append('<polygon points="' +
               " ".join(f"{x:.1f},{y:.1f}" for x, y in head) +
               '" fill="#475569"/>')


text(600, 45, "FOLLOW_AVOID 控制循环", 36)
card(70, 140, "ALIGN_TARGET", "每轮先按 AOA 转向", "#e0e7ff")
card(460, 140, "FORWARD", "前进并持续测距", "#dcfce7")
card(850, 140, "SCAN_SIDE", "遇障停车，选可走方向", "#e0f2fe")
card(850, 400, "TURN_OUT", "向可走方向转向", "#fef3c7")
card(460, 400, "ADVANCE", "低速绕行 150 mm", "#fef3c7")

arrow([(350, 200), (460, 200)])
arrow([(740, 200), (850, 200)])
arrow([(990, 260), (990, 400)])
arrow([(850, 460), (740, 460)])
arrow([(460, 460), (210, 460), (210, 260)])
arrow([(600, 140), (600, 105), (210, 105), (210, 140)])

text(405, 175, "对准后")
text(795, 175, "发现障碍")
text(1020, 330, "选路")
text(795, 435, "前方通畅")
text(330, 488, "绕行结束，再转向")
text(430, 85, "方向偏了，重新转向")
text(600, 590, "转向阶段不避障；进入前进阶段后，先确认新的前向测距再行驶。", 20)

svg.append("</svg>")
(OUT / "follow_avoid_flow.svg").write_text("\n".join(svg), encoding="utf-8")
image.save(OUT / "follow_avoid_flow.png")
