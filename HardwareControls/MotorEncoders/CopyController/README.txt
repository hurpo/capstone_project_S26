These files use your existing MotorController.py as the robot interface.

Files:
- motion_bridge.py
  Helper math that converts joystick/chassis commands into wheel commands while using
  MotorController.py's calibration and movement conventions.

- teleop_record.py
  Records joystick input, wheel commands, encoder counts, and estimated pose.

- replay_trace.py
  Replays a recorded trace in either:
    1) speed mode
    2) encoder-tracking mode
  and compares actual encoder counts against the recorded reference counts.

- run_steps.py
  Executes exported step files later.

Important replay note:
If the robot was oscillating back and forth during replay, the correction loop was likely too aggressive.
This updated version lowers the default encoder feedback gain and caps the correction speed.

Recommended first replay command:
    python3 replay_trace.py --trace teleop_trace.jsonl --mode encoder --kp-counts 0.10 --max-correction-rev-s 0.12 --rate 15

If that still oscillates, try:
    python3 replay_trace.py --trace teleop_trace.jsonl --mode encoder --kp-counts 0.05 --max-correction-rev-s 0.08 --rate 12

You can also test pure command replay:
    python3 replay_trace.py --trace teleop_trace.jsonl --mode speed
