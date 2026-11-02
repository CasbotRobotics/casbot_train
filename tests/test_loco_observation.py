"""Compare compiled deployment observation code with the supplied sim2sim.

Run: python3 tests/test_loco_observation.py /tmp/loco_observation_probe
Compile the probe with:
  g++ -std=c++17 -I . tests/loco_observation_probe.cpp -o /tmp/loco_observation_probe
Only numpy is needed; AST extraction avoids importing MuJoCo/training packages.
"""
import ast
from collections import deque
from pathlib import Path
import subprocess
import sys

import numpy as np


def main():
    root = Path(__file__).resolve().parents[1]
    reference = ast.parse((root / "sim2sim_casbot02_loco_onnx(1).py").read_text())
    names = {"gait_phase_features", "build_obs_term_major"}
    functions = [node for node in reference.body
                 if isinstance(node, ast.FunctionDef) and node.name in names]
    assert len(functions) == 2
    scope = {"np": np, "deque": deque, "TERM_DIMS": (3, 3, 3, 12, 12, 12)}
    exec(compile(ast.Module(body=functions, type_ignores=[]), "sim2sim", "exec"), scope)

    inputs, expected = [], []
    commands = [(0.5, 0, 0), (0, 0.2, 0), (0, 0, -0.5), (0, 0, 0),
                (1e-6, 0, 0), (1.0001e-6, 0, 0), (0, 0, 1e-6)]
    # First-frame fill, history sliding, standing/restarting, turns, threshold,
    # wraparound and an explicit clock restart. Distinct slots expose ordering.
    times = list(np.arange(0, 3, 0.02)) + [0.25, 0.5, 0.75, 1.0, 1000.125, 0.0]
    for phase_frames in (0, 1, 4):
        for scenario in range(len(commands) + 1):
            history, phase_history = deque(maxlen=4), deque(maxlen=4)
            for step, time in enumerate(times):
                # Last scenario changes command so history crosses walk/stop/turn.
                command = commands[scenario] if scenario < len(commands) else commands[(step // 5) % 4]
                reset = step == 0 or step == len(times) - 1
                if reset:
                    history.clear()
                    phase_history.clear()
                frame = (np.arange(45, dtype=np.float32) + step * 100) / 1000
                if not history:
                    history.extend([frame] * 4)
                else:
                    history.append(frame)
                phase = scope["gait_phase_features"](time, np.array(command)) if phase_frames else None
                if phase_frames == 4:
                    if not phase_history:
                        phase_history.extend([phase] * 4)
                    else:
                        phase_history.append(phase)
                    phase = np.asarray(phase_history)
                expected.append(scope["build_obs_term_major"](history, phase).ravel())
                values = [time, *command, phase_frames, int(reset), *np.asarray(history).ravel()]
                inputs.append(" ".join(str(value) for value in values))
    result = subprocess.run([sys.argv[1]], input="\n".join(inputs) + "\n",
                            text=True, capture_output=True, check=True)
    lines = result.stdout.splitlines()
    assert len(lines) == len(expected)
    for line, want in zip(lines, expected):
        got = np.fromstring(line, sep=" ", dtype=np.float32)
        assert got.shape == want.shape
        np.testing.assert_array_equal(got[:180], want[:180])
        np.testing.assert_allclose(got[180:], want[180:], rtol=0, atol=1e-7)
    print(f"PASS: {len(expected)} deployment inputs match sim2sim (180/184/196 dimensions)")


if __name__ == "__main__":
    main()
