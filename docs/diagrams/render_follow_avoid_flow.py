"""Render the current FOLLOW_AVOID state flow to SVG and PNG."""

from html import escape
from math import atan2, cos, pi, sin
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUT = Path(__file__).resolve().parent
WIDTH, HEIGHT = 2050, 1110
BG = "#f8fafc"
INK = "#132238"
LINE = "#64748b"
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

image = Image.new("RGB", (WIDTH, HEIGHT), BG)
draw = ImageDraw.Draw(image)
svg = [
    f'<svg xmlns="http://www.w3.org/2000/svg" width="{WIDTH}" '
    f'height="{HEIGHT}" viewBox="0 0 {WIDTH} {HEIGHT}">',
    f'<rect width="{WIDTH}" height="{HEIGHT}" fill="{BG}"/>',
]


def font(size, bold=False):
    return ImageFont.truetype(BOLD if bold else FONT, size)


def text_at(x, y, value, size=22, color=INK, bold=False):
    draw.text((x, y), value, font=font(size, bold), fill=color, anchor="mm")
    weight = "700" if bold else "400"
    svg.append(
        f'<text x="{x}" y="{y}" text-anchor="middle" '
        f'dominant-baseline="central" font-family="Noto Sans CJK SC" '
        f'font-size="{size}" font-weight="{weight}" fill="{color}">'
        f'{escape(value)}</text>'
    )


def rounded_box(x, y, w, h, fill, stroke, radius=20, width=3):
    draw.rounded_rectangle((x, y, x + w, y + h), radius, fill, stroke, width)
    svg.append(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{radius}" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>'
    )


def arrow(points, color=LINE, width=4, dashed=False):
    draw.line(points, fill=color, width=width, joint="curve")
    x1, y1 = points[-2]
    x2, y2 = points[-1]
    theta = atan2(y2 - y1, x2 - x1)
    head = [
        (x2, y2),
        (x2 - 15 * cos(theta - pi / 6), y2 - 15 * sin(theta - pi / 6)),
        (x2 - 15 * cos(theta + pi / 6), y2 - 15 * sin(theta + pi / 6)),
    ]
    draw.polygon(head, fill=color)
    data = " ".join(f"{x},{y}" for x, y in points)
    dash = ' stroke-dasharray="9 8"' if dashed else ""
    svg.append(
        f'<polyline points="{data}" fill="none" stroke="{color}" '
        f'stroke-width="{width}" stroke-linejoin="round"{dash}/>'
    )
    data = " ".join(f"{x:.1f},{y:.1f}" for x, y in head)
    svg.append(f'<polygon points="{data}" fill="{color}"/>')


def label(x, y, value, color=INK):
    size = 19
    box = draw.textbbox((0, 0), value, font=font(size))
    w = box[2] - box[0] + 26
    rounded_box(x - w / 2, y - 18, w, 36, "#ffffff", "#e2e8f0", 10, 1)
    text_at(x, y, value, size, color)


def state(x, y, title, lines, fill, stroke, h=150):
    rounded_box(x, y, 270, h, fill, stroke)
    compact = h < 130
    text_at(x + 135, y + (33 if compact else 40), title, 25, INK, True)
    for i, line in enumerate(lines):
        text_at(x + 135, y + (67 + i * 26 if compact else 83 + i * 29), line, 19)


# Main forward path and less common transitions, drawn behind the states.
arrow([(35, 405), (70, 405)])
for x in (340, 655, 970, 1285, 1600):
    arrow([(x, 405), (x + 45, 405)])

arrow([(205, 330), (205, 255), (745, 255), (745, 330)])
arrow([(1150, 330), (1150, 210), (830, 210), (830, 330)])
arrow([(1460, 330), (1460, 165), (905, 165), (905, 330)])

arrow([(835, 480), (835, 620)])
arrow([(1150, 480), (1150, 620)], "#dc2626")
arrow([(1465, 480), (1465, 580), (1295, 580), (1295, 675), (1285, 675)], "#dc2626")
arrow([(1015, 675), (980, 675), (980, 540), (950, 540), (950, 480)])
arrow([(700, 675), (355, 675), (355, 440), (340, 440)])
arrow([(1720, 480), (1720, 825), (995, 825), (995, 440), (970, 440)])
arrow([(1835, 480), (1835, 935), (520, 935), (520, 480)], "#16a34a")

# State cards.
state(70, 330, "WAIT_FRONT", ["停车，舵机回中", "等待有效前方测距"], "#e0f2fe", "#38bdf8")
state(385, 330, "TRACK", ["需前进时轮询 0°、±5°", "换角等待 60 ms，继续行驶"], "#dcfce7", "#4ade80")
state(700, 330, "SCAN_SIDE", ["停车扫描左右", "寻找可绕行方向"], "#e0f2fe", "#38bdf8")
state(1015, 330, "TURN_OUT", ["回中、转向、停稳、测距", "整次动作限时 5 秒"], "#fef3c7", "#f59e0b")
state(1330, 330, "ADVANCE", ["低速前进", "每段最多 150 mm"], "#fef3c7", "#f59e0b")
state(1645, 330, "TRY_REJOIN", ["按最新 AOA 原地对准", "转向没有 5 秒限制"], "#ede9fe", "#a78bfa")
state(700, 620, "RESCAN_WAIT", ["停车等待 2 秒", "再判断前方"], "#e0f2fe", "#38bdf8", 110)
state(1015, 620, "WAIT_FRONT 恢复", ["停车，舵机回中", "新前方测距后重扫"], "#e0f2fe", "#38bdf8", 110)

# Conditions are on white pills so return arrows remain easy to follow.
for x, value in (
    (362, "≥ 650"),
    (677, "任一探测方向 ＜ 350"),
    (992, "侧向 ≥ 800"),
    (1307, "新方向 ≥ 650"),
    (1622, "前进 150 mm"),
):
    label(x, 520, value)

label(475, 238, "前方 ＜ 650，仍需接近目标")
label(990, 193, "新方向受阻或转角过冲")
label(1250, 148, "前方 ＜ 350，重新扫描")
label(835, 565, "无通路 / 扫描超时")
label(1150, 565, "整次转向满 5 秒", "#b91c1c")
label(1380, 563, "前进卡住", "#b91c1c")
label(435, 658, "等待结束")
label(1370, 806, "对准后，新测距 ＜ 650：重新绕行")
label(1190, 916, "对准后，两次不同新测距均 ≥ 650：恢复跟随", "#166534")

text_at(570, 57, "FOLLOW_AVOID 当前避障跟随流程", 38, INK, True)
text_at(493, 103, "前方距离单位：mm  ·  箭头表示状态切换", 22, "#475569")

rounded_box(70, 982, 1845, 102, "#eef2f7", "#cbd5e1", 15, 1)
text_at(992, 1007, "TRY_REJOIN：AOA 偏差 ＞ 12° 时持续原地转；测距无效则停车等待。", 20)
text_at(992, 1034, "TRACK 连续 250 ms 无新测距则停车回中；TURN_OUT / ADVANCE 测距失效也回 WAIT_FRONT。", 20)
text_at(992, 1061, "AOA 失效则停车回 WAIT_FRONT；目标够近时保持 0°；只有配置错误等不可恢复故障进入 HOLD。", 20)

svg.append("</svg>")
(OUT / "follow_avoid_flow.svg").write_text("\n".join(svg), encoding="utf-8")
image.save(OUT / "follow_avoid_flow.png")
