"""Read-only quantitative comparison of two wave EMI traces."""

from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
STEMS = (
    "wave_20260930_113102_232672377",
    "wave_20260930_145627_332661118",
)


def smoothed_speed(t, q, half_window=0.10):
    return (np.interp(t + half_window, t, q) - np.interp(t - half_window, t, q)) / (2 * half_window)


def runs(mask, times):
    indices = np.flatnonzero(mask)
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [(round(float(times[g[0]]), 3), round(float(times[g[-1]] - times[g[0]]), 3))
            for g in groups if len(g) >= 5]


for stem in STEMS:
    cmd = pd.read_csv(ROOT / (stem + ".csv"))
    fb = pd.read_csv(ROOT / (stem + "_feedback.csv"))
    t = cmd["send_end_s"].to_numpy(float)
    phase = cmd["phase"].to_numpy()
    tf = fb["feedback_time_s"].to_numpy(float)
    print("RUN", stem)
    print("rows", len(cmd), "feedback", len(fb), "phases", cmd["phase"].value_counts().to_dict())
    print("feedback_error_codes", fb["error_code"].value_counts().to_dict())
    for name in ("起势", "主体"):
        mask = phase == name
        gap = np.diff(t[mask]) * 1000
        print("send", name, "mean_ms", round(float(np.mean(gap)), 3),
              "p95_ms", round(float(np.quantile(gap, 0.95)), 3),
              "max_ms", round(float(np.max(gap)), 3),
              "gt25ms", int(np.sum(gap > 25)))
    for axis in (1, 3, 4):
        target = cmd[f"target_J{axis}"].to_numpy(float)
        actual = np.interp(t, tf, fb[f"actual_J{axis}"].to_numpy(float))
        error = target - actual
        vt = smoothed_speed(t, target)
        va = smoothed_speed(t, actual)
        print("joint", axis)
        for name in ("起势", "主体"):
            mask = phase == name
            em = np.abs(error[mask])
            idx = np.flatnonzero(mask)[np.argmax(em)]
            print("  ", name, "err_p95", round(float(np.quantile(em, 0.95)), 3),
                  "err_max", round(float(em.max()), 3),
                  "at_t", round(float(t[idx]), 3),
                  "target_v_max", round(float(np.max(np.abs(vt[mask]))), 2),
                  "actual_v_max", round(float(np.max(np.abs(va[mask]))), 2))
        if axis in (1, 4):
            mask = (phase == "起势") & (np.abs(vt) > 10) & (np.abs(va) < np.abs(vt) * 0.5)
            print("  rise_velocity_dips", runs(mask, t))
            raw = fb[f"speed_raw_J{axis}"].to_numpy(float) * 180.0 / np.pi
            raw_at_cmd = np.interp(t, tf, raw)
            mask_raw = ((phase == "起势") & (np.abs(vt) > 10)
                        & (np.abs(raw_at_cmd) < np.abs(vt) * 0.5))
            print("  rise_raw_speed_dips", runs(mask_raw, t),
                  "raw_speed_peak", round(float(np.max(np.abs(raw_at_cmd[phase == "起势"]))), 2))
        if axis == 3:
            body = phase == "主体"
            body_t = t[body] - t[body][0]
            body_e = np.abs(error[body])
            print("  body_error_gt10deg", runs(body_e > 10, body_t))
            body_core = body & (t > t[body][0] + 0.3) & (t < t[body][-1] - 0.3)
            lags = np.arange(0.0, 0.501, 0.005)
            rmses = [np.sqrt(np.mean((np.interp(t[body_core] - lag, t, target)
                                     - actual[body_core]) ** 2)) for lag in lags]
            best = int(np.argmin(rmses))
            print("  body_j3_lag_fit_s", round(float(lags[best]), 3),
                  "rmse_zero", round(float(rmses[0]), 3),
                  "rmse_best", round(float(rmses[best]), 3))

first = pd.read_csv(ROOT / (STEMS[0] + ".csv"))
second = pd.read_csv(ROOT / (STEMS[1] + ".csv"))
a = first[first.phase == "主体"].reset_index(drop=True)
b = second[second.phase == "主体"].reset_index(drop=True)
n = min(len(a), len(b))
print("BODY_TARGET_RUN_DIFFERENCE", {
    f"J{axis}": round(float(np.max(np.abs(
        a[f'target_J{axis}'].to_numpy(float)[:n]
        - b[f'target_J{axis}'].to_numpy(float)[:n]
    ))), 6)
    for axis in range(1, 8)
})
