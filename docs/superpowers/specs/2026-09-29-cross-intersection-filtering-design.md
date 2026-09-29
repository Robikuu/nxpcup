# Design Specification: Gated Cross-Intersection Filtering & Straight-Through Transit

## 1. Overview & Problem Statement

On the NXP Cup track, the intersection is a standard $90^\circ$ cross-junction:
```text
  _|      |_
  _        _
   |      |
```
When approaching and traversing this intersection at full speed:
1. **Horizontal Line Pull:** Perpendicular road markings (`_  _`) extend horizontally across the track ($|\text{angle}| \approx 70^\circ\dots 90^\circ$). If not rejected, the lane-centering algorithm will misidentify these crossbars as boundary lines and turn the steering servo into the cross-street.
2. **Boundary Discontinuity:** Inside the cross-junction box, lateral road boundary lines end for $\approx 40\text{–}50\,\text{cm}$. The vehicle must not slow down or initiate sharp cornering maneuvers while in the box; it must hold its course dead-straight across the junction at $100\%$ full speed and re-acquire lane boundaries on the opposite side.

---

## 2. Architecture & Control Logic

```
                    [CAMERA VECTORS ACQUIRED]
                                │
                                ▼
               Angle Rejection Filter (|angle| > 60°)
              /                                     \
        YES  /                                       \  NO
            /                                         \
[Flag Transverse Tape]                          [Normal Lane Centering]
Reject vector from left/right                     Compute closest left
boundary selection.                               and right boundaries.
            │                                         │
            └───────────────────┬─────────────────────┘
                                │
                                ▼
         Check Intersection Entry Conditions:
         - sr.has_transverse_tape == 1
         - Vehicle aligned straight (|Heading| <= 12°, |Steer| <= 12°)
         - Currently in MACRO_STATE_NORMAL
                                │
                                ▼
                 [ENTER MACRO_STATE_INTERSECTION]
                 - State: ST: I
                 - Speed: 100% full throttle sprint (both wheels)
                 - Steering: Hold 0.0° dead straight
                 - Transit window: Min 8 ticks (160ms)
                                │
                                ▼
         Exit Conditions (Return to MACRO_STATE_NORMAL):
         - Transit ticks >= 8 AND 2 clean longitudinal lines re-acquired
           (sr.found == 2) for 2 consecutive ticks, OR
         - Timeout backstop reached (22 ticks = 440ms)
```

---

## 3. Detailed Component Modifications

### 3.1 [`source/vectors.h`](file:///d:/nxpcup/source/vectors.h)
- Add `uint8_t has_transverse_tape;` to `steering_result_t`.

### 3.2 [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c)
- In `compute_top_camera_lane()`:
  - If $|\text{angle}| > 60.0^\circ$:
    - Check if vector is a valid crossbar: $|\Delta x| \ge 12.0\,\text{px}$ and $y_{\text{near}} \ge 16.0\,\text{px}$.
    - Set `out->has_transverse_tape = 1;`
    - Execute `continue;` so it is **never** considered as a left or right lane boundary.

### 3.3 [`source/main.c`](file:///d:/nxpcup/source/main.c)
- Define macro states:
  ```c
  #define MACRO_STATE_NORMAL       0
  #define MACRO_STATE_INTERSECTION 1
  ```
- **Intersection Trigger:**
  ```c
  if (macro_state == MACRO_STATE_NORMAL) {
      if (sr.has_transverse_tape && fabsf(filtered_heading) <= 12.0f && fabsf(steer_cmd) <= 12.0f) {
          macro_state = MACRO_STATE_INTERSECTION;
          intersection_ticks = 0;
          intersection_reacquire_streak = 0;
      }
  }
  ```
- **In `MACRO_STATE_INTERSECTION`:**
  - **Steering:** Command `steer_cmd = 0.0f;` (dead straight).
  - **Speed:** Command `target_speed = g_params.speed_straight; target_left = target_speed; target_right = target_speed;` ($100\%$ full speed, zero differential drag).
  - **Exit Condition:**
    ```c
    if (++intersection_ticks >= 8) {
        if (sr.found == 2) {
            if (++intersection_reacquire_streak >= 2) {
                macro_state = MACRO_STATE_NORMAL;
            }
        } else {
            intersection_reacquire_streak = 0;
        }
    }
    if (intersection_ticks >= 22) {
        macro_state = MACRO_STATE_NORMAL;
    }
    ```
- **Telemetry:**
  - Log `ST: I` while in the intersection.
