# Manual gripper test

From the repository root, in DroidAuto's own environment:

```bash
python scripts/gripper_commands.py
```

The endpoint comes from `configs/station.local.json` if present, otherwise
`configs/station.example.json`. Override it with `--endpoint tcp://HOST:4242`.
The project must be installed in this environment (`python -m pip install --no-deps -e .`).

Type a choice and press **Enter**:

| Choice | Action |
| --- | --- |
| `1` / `o` / `open` | Send one blocking open command. |
| `2` / `c` / `close` | Send one blocking close command. |
| `3` / `s` / `state` | Print the position reported by DROID: 0 open, 1 closed. |
| `h` | Show the menu. |
| `q` / `0` | Exit without a gripper command. |

An empty Enter does nothing. The screen never clears or automatically refreshes.
There is no automatic movement cycle. Command return and reported position are
shown separately; a returned RPC alone does not establish physical movement or
prove that feedback is live. Readout does not depend on arm-state age checks.
Errors return to the menu; commands are never automatically retried. Full
tracebacks and command history are recorded under `runs/gripper_commands/`.
Ctrl+C at the prompt exits; Ctrl+C during a request exits with its outcome
unknown. Quitting sends no cleanup movement or controller shutdown.

If controller startup is needed, explicitly request it:

```bash
python scripts/gripper_commands.py --initialize-controllers
```

This relaunches **both arm and gripper controllers** through the existing DROID
startup sequence. The default invocation only connects; it does not relaunch
controllers. The test uses `DroidRobot.open_gripper()`, `close_gripper()` and
`read_gripper_position()` and sends no arm-position commands.
