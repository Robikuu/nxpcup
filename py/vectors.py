#!/usr/bin/env python3
"""
Live visualization of Pixy vector data streamed from print_frame_debug()
over serial. Two panels: raw pixel-space (mimics the camera view, PixyMon-
style) and homography-transformed ground-space (top-down / bird's-eye).

Reality check: the display can only update as fast as full frames actually
arrive over serial. print_frame_debug() does several blocking esp_printf
calls per vector, every Pixy frame -- we already established that slows
the ESP's own loop down. The drawing loop here is capped at 60fps as a
CEILING, but what you actually see will be however fast real frames show
up, which is very likely well under that. That's expected, not a bug in
this script.

Install deps:
    pip install pyserial matplotlib

Usage:
    python live_vectors.py --port COM5 --baud 115200
    (use your actual COM port / baud rate)
"""

import argparse
import re
import queue
import threading

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ---- adjust these to match your actual setup ----
PIXEL_FRAME_W = 79     # Pixy2's native line-tracking grid width -- documented (pixy.frameWidth in
                       # line-tracking mode), not a guess this time. Was wrongly set to 640 before,
                       # which is why vectors looked squeezed into a tiny corner of this panel.
PIXEL_FRAME_H = 52     # same -- pixy.frameHeight in line-tracking mode
GROUND_X_RANGE = (-40, 40)   # cm, slightly wider than the +-35 bound in pixel_to_ground
GROUND_Y_RANGE = (0, 115)    # cm, slightly wider than the 5-110 bound in pixel_to_ground


# ---- line formats, matching print_frame_debug()'s exact esp_printf output ----
VECTOR_RE    = re.compile(r'^Vector \[(\d+)\] details:')
PIXEL_RE     = re.compile(r'^\s*Pixel:\s*\((\d+),\s*(\d+)\)\s*->\s*\((\d+),\s*(\d+)\)')
GSTART_RE    = re.compile(r'^\s*Ground start:\s*\((-?\d+\.\d+),\s*(-?\d+\.\d+)\)\s*cm')
GEND_RE      = re.compile(r'^\s*Ground end:\s*\((-?\d+\.\d+),\s*(-?\d+\.\d+)\)\s*cm')
GOOR_RE      = re.compile(r'^\s*Ground:\s*OUT OF CALIBRATED RANGE')
FOUND_RE     = re.compile(r'^Boundary vectors selected:\s*(\d+)')
CTE_RE       = re.compile(r'^\s*Center cte:\s*(-?\d+\.\d+)\s*cm')
HEAD_RE      = re.compile(r'^\s*Center heading:\s*(-?\d+\.\d+)\s*deg')
NOOVERLAP_RE = re.compile(r'^\s*Center:\s*boundaries found but no Y overlap')
FRAMEEND_RE  = re.compile(r'^---FRAME_END---')


class FrameParser:
    """Feed it lines one at a time; get back a complete frame dict whenever
    a ---FRAME_END--- marker closes one out. Unrecognized lines (e.g. your
    main loop's 'L:.. R:.. F:.. S:..' debug line, boot messages) are
    silently ignored -- this only reacts to lines it recognizes, so
    anything else interleaved in main.c's output doesn't confuse it."""

    def __init__(self):
        self._reset()

    def _reset(self):
        self.vectors = []
        self.current = None
        self.boundary_found = None
        self.center_cte = None
        self.center_heading = None
        self.center_valid = None

    def feed_line(self, line):
        line = line.rstrip('\r\n')

        m = VECTOR_RE.match(line)
        if m:
            if self.current is not None:
                self.vectors.append(self.current)
            self.current = {'idx': int(m.group(1)), 'pixel': None, 'ground': None}
            return None

        if self.current is not None:
            m = PIXEL_RE.match(line)
            if m:
                self.current['pixel'] = tuple(int(g) for g in m.groups())
                return None
            m = GSTART_RE.match(line)
            if m:
                self.current['ground'] = self.current.get('ground') or {}
                self.current['ground']['start'] = (float(m.group(1)), float(m.group(2)))
                return None
            m = GEND_RE.match(line)
            if m:
                self.current['ground'] = self.current.get('ground') or {}
                self.current['ground']['end'] = (float(m.group(1)), float(m.group(2)))
                return None
            if GOOR_RE.match(line):
                self.current['ground'] = None
                return None

        m = FOUND_RE.match(line)
        if m:
            self.boundary_found = int(m.group(1))
            return None

        m = CTE_RE.match(line)
        if m:
            self.center_cte = float(m.group(1))
            self.center_valid = True
            return None

        m = HEAD_RE.match(line)
        if m:
            self.center_heading = float(m.group(1))
            return None

        if NOOVERLAP_RE.match(line):
            self.center_valid = False
            return None

        if FRAMEEND_RE.match(line):
            if self.current is not None:
                self.vectors.append(self.current)
                self.current = None
            frame = {
                'vectors': self.vectors,
                'boundary_found': self.boundary_found,
                'center_cte': self.center_cte,
                'center_heading': self.center_heading,
                'center_valid': self.center_valid,
            }
            self._reset()
            return frame

        return None


def serial_reader(ser, frame_queue, stop_event):
    parser = FrameParser()
    while not stop_event.is_set():
        try:
            raw = ser.readline()
        except serial.SerialException:
            break
        if not raw:
            continue
        try:
            line = raw.decode('utf-8', errors='replace')
        except Exception:
            continue

        frame = parser.feed_line(line)
        if frame is not None:
            # Keep only the newest frame in the queue -- if the plotting
            # side falls behind, drop stale frames instead of building up
            # lag over the course of a run.
            try:
                while True:
                    frame_queue.get_nowait()
            except queue.Empty:
                pass
            frame_queue.put(frame)


def make_plot():
    fig, (ax_px, ax_gnd) = plt.subplots(1, 2, figsize=(12, 6))

    ax_px.set_title('Pixel space (camera view)')
    ax_px.set_xlim(0, PIXEL_FRAME_W)
    ax_px.set_ylim(0, PIXEL_FRAME_H)
    ax_px.invert_yaxis()  # image coords: v grows downward, flip so it looks like the camera view
    ax_px.set_aspect('equal')

    ax_gnd.set_title('Ground space (top-down, cm)')
    ax_gnd.set_xlim(*GROUND_X_RANGE)
    ax_gnd.set_ylim(*GROUND_Y_RANGE)
    ax_gnd.set_aspect('equal')
    ax_gnd.set_xlabel('X (lateral, cm)')
    ax_gnd.set_ylabel('Y (forward, cm)')
    ax_gnd.axvline(0, color='0.85', linewidth=1, zorder=0)  # car centerline reference

    status_text = ax_gnd.text(0.02, 0.98, '', transform=ax_gnd.transAxes,
                               va='top', fontsize=9, family='monospace')

    fig.tight_layout()
    return fig, ax_px, ax_gnd, status_text


def redraw(frame, ax_px, ax_gnd, status_text, artists):
    for a in artists:
        a.remove()
    artists.clear()

    colors = plt.cm.tab10.colors

    for v in frame['vectors']:
        color = colors[v['idx'] % len(colors)]

        if v['pixel'] is not None:
            x0, y0, x1, y1 = v['pixel']
            line, = ax_px.plot([x0, x1], [y0, y1], color=color, linewidth=2)
            artists.append(line)

        if v['ground'] is not None and 'start' in v['ground'] and 'end' in v['ground']:
            X0, Y0 = v['ground']['start']
            X1, Y1 = v['ground']['end']
            line, = ax_gnd.plot([X0, X1], [Y0, Y1], color=color, linewidth=2, marker='o', markersize=3)
            artists.append(line)

    status = f"vectors: {len(frame['vectors'])}   boundaries: {frame['boundary_found']}"
    if frame['center_valid']:
        status += f"\ncte: {frame['center_cte']:+.2f} cm  heading: {frame['center_heading']:+.2f} deg"
    elif frame['center_valid'] is False:
        status += "\ncenter: no Y overlap"
    status_text.set_text(status)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True, help='e.g. COM5 or /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.5)

    frame_queue = queue.Queue()
    stop_event = threading.Event()
    reader_thread = threading.Thread(
        target=serial_reader, args=(ser, frame_queue, stop_event), daemon=True
    )
    reader_thread.start()

    fig, ax_px, ax_gnd, status_text = make_plot()
    artists = []
    last_frame = {'vectors': [], 'boundary_found': None, 'center_cte': None,
                  'center_heading': None, 'center_valid': None}

    def update(_):
        nonlocal last_frame
        try:
            last_frame = frame_queue.get_nowait()
        except queue.Empty:
            pass  # no new data yet -- keep showing the last frame, don't block
        redraw(last_frame, ax_px, ax_gnd, status_text, artists)
        return artists + [status_text]

    # interval is a ceiling (60fps) on the DRAWING side only -- actual
    # update rate is however fast frames arrive over serial, see module
    # docstring.
    ani = animation.FuncAnimation(fig, update, interval=1000 // 60, blit=False, cache_frame_data=False)

    try:
        plt.show()
    finally:
        stop_event.set()
        ser.close()


if __name__ == '__main__':
    main()