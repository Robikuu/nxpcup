# DRV8871 Output Plant Gain Normalization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement a unified output plant gain scale (`DRIVE_SCALE = 0.80f`) with floating-point duty calculation and live Wi-Fi tuning to match the historical vehicle speed and torque baseline after the DRV8871 driver upgrade.

**Architecture:** Commanded motor speeds remain in their calibrated units throughout the controller and mixing pipeline. Right before hardware actuation, `HbridgeSpeed()` scales left and right speed demands by `g_params.drive_scale` using full floating-point precision, rounding directly to CTIMER timer ticks to eliminate integer truncation. Telemetry logs the pre-scale command and active scale factor.

**Tech Stack:** C99, NXP MCXN947 (Dual Cortex-M33 @ 150 MHz), MCUXpresso IDE GCC Toolchain, CTIMER0 PWM.

---

### Task 1: Interface & Header Updates

**Files:**
- Modify: [`include/Config.h:26-30`](file:///d:/nxpcup/include/Config.h#L26-L30)
- Modify: [`include/Config.h:120-130`](file:///d:/nxpcup/include/Config.h#L120-L130)
- Modify: [`include/hbridge.h:34`](file:///d:/nxpcup/include/hbridge.h#L34)

- [ ] **Step 1: Add `DEFAULT_DRIVE_SCALE` and update `robot_params_t` struct in `include/Config.h`**
  ```c
  #define DEFAULT_DRIVE_SCALE          0.80f     // Output plant gain scale (starting guess: 0.76 - 0.84)
  ```
  Add `float drive_scale;` member to `robot_params_t`.

- [ ] **Step 2: Update `HbridgeSpeed()` prototype in `include/hbridge.h`**
  ```c
  void HbridgeSpeed(Hbridge *h, float speed_left, float speed_right);
  ```

- [ ] **Step 3: Commit header changes**
  ```bash
  git add include/Config.h include/hbridge.h
  git commit -m "refactor: add drive_scale parameter and float HbridgeSpeed prototype"
  ```

---

### Task 2: Floating-Point Actuation in `source/hbridge.c`

**Files:**
- Modify: [`source/hbridge.c:32-75`](file:///d:/nxpcup/source/hbridge.c#L32-L75)

- [ ] **Step 1: Implement float precision scaling and CTIMER tick resolution**
  Update `HbridgeSpeed(Hbridge *h, float speed_left, float speed_right)`:
  - Clamp inputs to `[-100.0f, +100.0f]`.
  - Scale:
    ```c
    float scaled_left  = speed_left  * g_params.drive_scale;
    float scaled_right = speed_right * g_params.drive_scale;
    ```
  - Compute duty cycle in float:
    - Forward (`scaled > 0.0f`): `duty_float = 100.0f - scaled;`
    - Reverse (`scaled < 0.0f`): `duty_float = (mag * mag) / 100.0f;`
    - Stop (`scaled == 0.0f`): `duty_float = 0.0f;`
  - Compute CTIMER match registers directly:
    ```c
    uint32_t period = h->pwmPeripheral->MR[h->periodChannel];
    uint32_t pulse_left = (duty_left <= 0.0f) ? (period + 1U) :
                          (uint32_t)((float)period * (100.0f - duty_left) / 100.0f + 0.5f);
    h->pwmPeripheral->MR[h->pwm2Channel] = pulse_left;
    ```

- [ ] **Step 2: Commit `source/hbridge.c` changes**
  ```bash
  git add source/hbridge.c
  git commit -m "feat: implement float scaling and tick-accurate PWM in HbridgeSpeed"
  ```

---

### Task 3: Command Shell & Live Tuning in `source/wifi_config.c`

**Files:**
- Modify: [`source/wifi_config.c:17-40`](file:///d:/nxpcup/source/wifi_config.c#L17-L40)
- Modify: [`source/wifi_config.c:160-220`](file:///d:/nxpcup/source/wifi_config.c#L160-L220)

- [ ] **Step 1: Initialize parameter in `params_init()`**
  ```c
  g_params.drive_scale = DEFAULT_DRIVE_SCALE;
  ```

- [ ] **Step 2: Add `scale` command to Wi-Fi command parser**
  Support keywords `scale` and `drive_scale` with range validation `[0.10f, 1.00f]`.

- [ ] **Step 3: Update `status` output and `help` text**
  Include `scale` in the status printout and command list.

- [ ] **Step 4: Commit `source/wifi_config.c` changes**
  ```bash
  git add source/wifi_config.c
  git commit -m "feat: add scale live tuning command to wifi shell"
  ```

---

### Task 4: Main Loop & Telemetry Integration in `source/main.c`

**Files:**
- Modify: [`source/main.c:605-640`](file:///d:/nxpcup/source/main.c#L605-L640)

- [ ] **Step 1: Pass float speeds directly to `HbridgeSpeed()`**
  ```c
  HbridgeSpeed(&g_hbridge, applied_speed_left, applied_speed_right);
  ```

- [ ] **Step 2: Append scale value to ASCII telemetry stream**
  Update telemetry buffer formatting:
  ```c
  snprintf(telem_buf, sizeof(telem_buf),
           "L:%d/%d R:%d F:%d H:%d TS:%d FS:%d S:%d BW:%d DOM:%c FV:%d IX:%d ST:%c VL:%d VR:%d SC:%d\r\n",
           ...,
           (int)applied_speed_left, (int)applied_speed_right,
           (int)(g_params.drive_scale * 100.0f));
  ```

- [ ] **Step 3: Commit `source/main.c` changes**
  ```bash
  git add source/main.c
  git commit -m "feat: log drive scale in telemetry stream"
  ```

---

### Task 5: Compilation, Synchronization & Verification

**Files:**
- Sync: `source/*`, `include/*` to `D:\robotica\workspace\nxpcup-ciorba_curata`
- Output: `d:\nxpcup\Debug\nxpcup.axf`, `d:\nxpcup\Debug\nxpcup.map`

- [ ] **Step 1: Sync modified source and header files to workspace**
- [ ] **Step 2: Execute MCUXpresso headless build**
  Verify 0 errors, 0 warnings.
- [ ] **Step 3: Copy generated binary and map file to repository `Debug/`**
- [ ] **Step 4: Confirm binary timestamp and size**
