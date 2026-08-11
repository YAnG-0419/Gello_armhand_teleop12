# FR3 Arm UI

`arm_ui` is the operator-facing, single-arm teach-and-repeat application. It
keeps the browser separate from ROS control: the UI edits named joint
waypoints and routines, while `ArmRosRuntime` alone reads fresh robot state,
switches controllers, and submits a MoveIt Pilz sequence.

## Workflow

1. Select the left or right arm.
2. Click **进入零力矩拖动**, support the arm, and guide it by hand.
3. Click **记录当前位置**, name the captured seven-joint configuration, and
   save it. Editing degrees only changes the saved target; it never commands
   the robot.
4. Drag 1–9 named points into the task list and set velocity, acceleration, and
   blend radius.
5. Click **START**. The backend reads a fresh joint state, switches that arm to
   its trajectory controller, and sends one `MoveGroupSequence` goal. The arm
   stays under trajectory control at the final point.
6. Click **进入零力矩拖动** explicitly before guiding the arm again.

`STOP` requests cancellation through the MoveIt action. It is not a hardware
emergency stop.

## Storage

Files are written atomically under `/data/arm_ui` in the container:

- `waypoints.yaml`: named left/right joint configurations in radians.
- `routines.yaml`: named sequences referencing those point names, plus speed,
  acceleration, and blend settings.

The browser also exposes download buttons for both files. The `/data` volume is
provided by `TELEOP_DATA_ROOT` in `docker/.env`.

## Start

The normal MoveIt launcher starts the UI together with fake or real hardware:

```bash
./ops/run/run_moveit.sh --fake
./ops/run/run_moveit.sh --real
```

Open <http://localhost:8081>. Fake mode supports point editing and playback,
but its velocity-interface controller configuration does not provide the
zero-effort teach controller. Hand-guiding is a real-hardware feature.

On a graphical workstation `run_moveit.sh` opens the page automatically after
startup. Set `ARM_UI_NO_BROWSER=1` when running unattended or when browser
launching is not wanted.

The real configuration uses one controller manager per arm. The teach and arm
controllers claim the same seven effort interfaces and therefore can never be
active together. The runtime uses strict controller switches and refuses a
mode switch while a trajectory action is active. The UI also permits only one
arm in zero-effort mode at a time; exit hand guiding and hold that arm before
selecting the other side or starting a task.

Intermediate points use the configured Pilz blend radius. A non-zero radius
means continuous motion passes near the intermediate target instead of
stopping exactly on it; the final point always has zero blend and is held.
