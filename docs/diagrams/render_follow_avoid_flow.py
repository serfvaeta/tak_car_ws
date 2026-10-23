"""Render the current FOLLOW_AVOID cycle to SVG and PNG."""

from html import escape
from math import atan2, cos, pi, sin
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
WIDTH, HEIGHT = 1000, 1030
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


text(500, 45, "FOLLOW_AVOID 三层选角与绕行", 34)
card(350, 100, "ALIGN_TARGET", "按 AOA 对准目标", "#e0e7ff")
card(350, 235, "FORWARD", "跟随并持续探测", "#dcfce7")
card(350, 370, "粗选角", "+90° 到 -90°，每 15°", "#e0f2fe")
card(350, 505, "细选角", "粗选角 ±20°，每 5°", "#dbeafe")
card(350, 640, "实际转向角", "细选角向外偏 16°", "#fef3c7")
card(350, 775, "ADVANCE", "min(细选测距, 2000) - 200 mm", "#fef3c7")
card(40, 505, "RESCAN_WAIT", "停车 2 秒后重新粗扫", "#fee2e2")

arrow([(490, 220), (490, 235)])
arrow([(490, 355), (490, 370)])
arrow([(490, 490), (490, 505)])
arrow([(490, 625), (490, 640)])
arrow([(490, 760), (490, 775)])
arrow([(490, 895), (900, 895), (900, 75), (490, 75), (490, 100)])
arrow([(350, 430), (300, 430), (300, 565), (320, 565)])
arrow([(350, 565), (320, 565)])
arrow([(350, 700), (300, 700), (300, 565), (320, 565)])
arrow([(320, 565), (280, 565), (280, 430), (350, 430)])

text(770, 225, "对准后")
text(770, 360, "发现障碍")
text(770, 495, "找到粗选角")
text(790, 630, "按距离与 AOA 细选")
text(790, 765, "正前方测距放行")
text(760, 925, "绕行结束，重新对准")
text(180, 650, "无合格路 / 超时 / 转向受阻", 17)
text(500, 970, "粗扫超时 7 秒；细扫超时 5 秒；扫描期间底盘停车。", 19)

svg.append("</svg>")
(OUT / "follow_avoid_flow.svg").write_text("\n".join(svg), encoding="utf-8")
image.save(OUT / "follow_avoid_flow.png")
