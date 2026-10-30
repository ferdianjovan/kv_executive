# October 27 executive, monitoring and perception SIGSEGV

The package launch files now use a shared-library Python launcher permanently.
The earlier opt-in launch argument and active rollback control were retired
after the user reported that the segmentation faults stopped. This does not change
`verification.py`, motion commands, REACQUIRE viewpoints, detector persistence,
flight timings, middleware selection or message definitions. It does not catch
SIGSEGV, suppress an exception, resume a failed process or restart a flight node.

## Findings

The production crash dumps are in
`/mnt/c/Users/s04fj3/AppData/Local/Temp/wsl-crashes/`:

| Process | Dump | Interrupted instruction | Actual destination | Original CR2 |
| --- | --- | --- | --- | --- |
| Executive, left REACQUIRE move | `wsl-crash-1793135351-5610-_usr_bin_python3.12-11.dmp` | `0x5e3259` | `0xba676c` | `0x40000ba676c` |
| Earlier executive, startup | `wsl-crash-1793135024-5569-_usr_bin_python3.12-11.dmp` | `0x5e3259` | `0xba676c` | `0x40000ba676c` |
| Detection monitor | `wsl-crash-1793135366-5522-_usr_bin_python3.12-11.dmp` | `0x5d66ba` | `0xba676c` | `0x40000ba676c` |
| Thermal fire detector | `wsl-crash-1793137181-10994-_usr_bin_python3.12-11.dmp` | `0x5e3259` | `0xba676c` | `0x40000ba676c` |
| RGB smoke detector | `wsl-crash-1793137313-10993-_usr_bin_python3.12-11.dmp` | `0x5e3259` | `0xba676c` | `0x40000ba676c` |

All five interrupted instructions store CPython's recursion counter into
valid writable interpreter memory. The original fault address is exactly
4 TiB higher than the instruction's computed destination. The original CR2
was recovered from the saved Linux signal context: the visible core signal
can instead describe Python's fatal-handler re-raise. The monitor also crashed
again while its fatal handler printed the traceback.

The earlier executive died about 5.8 seconds after launch, before a mission.
A fresh plain ROS worker reproduced this same native signature after 45.30
seconds on isolated domain 220. That worker imports only standard ROS interfaces
and no mission, verification, perception or custom interface modules. This
establishes that the reported flight sequence and geographical location are
not required for this fault.

The executive's worker was waiting for movement while its main executor thread
crashed. The initial REACQUIRE climb/left branch still uses ordinary PlanToPose;
it had not entered the new inspection planner or thermal localizer. The executor
entrypoint is unchanged. Repeated serialization checks of the installed custom
messages and services passed. The production core library paths use the intended
workspace, and both affected nodes already used UDP-only Fast DDS without its
shared-memory mappings.

The evidence does not identify which runtime, native library, kernel or host
component caused the inconsistent address. It does not establish that a WSL
upgrade, a different middleware, affinity or the launcher below repairs it.
No system or Windows settings were changed for this investigation.

The later detector crashes occurred while the user was running the shared
executive successfully through climb, left and right viewpoint moves. That
executive exited normally after about 468 seconds. Both detector cores still
execute the stock interpreter at fixed low addresses in executor polling;
OpenCV/TBB workers are waiting. The detector cores happen to map libpython via
native dependencies already, so a library mapping alone is not proof that the
shared runtime was active. Tests check the native launcher executable and active
CPython runtime addresses as well. The additional native evidence is retained
under `evidence/perception_followup/`.

## Current launch behavior

After rebuilding `kv_perception` and `kv_executive` and sourcing the workspace,
use the normal commands with your usual camera and mission arguments:

```bash
ros2 launch kv_executive autonomy.launch.py
ros2 launch kv_executive monitoring.launch.py
ros2 launch kv_perception perception.launch.py
```

The standalone `detection_monitor.launch.py` uses the same permanent runtime.
The executive, detection monitor and both detectors print `[kv_shared_python]` at startup.
The planner and native battery monitor remain their normal native executables.
Perception installs its own copy of the same native launcher and package-local
prefix helper. It can be built independently and does not introduce a circular
dependency on the executive.

`python_shared_runtime` links the already installed `libpython3.12.so.1.0`
instead of running Ubuntu's statically linked `/usr/bin/python3.12`. The Python
version remains 3.12.3 on the investigated machine. This changes interpreter
code, linking and memory layout; it is not evidence that ASLR alone fixes the
fault. It uses Python's supported PyConfig/Py_RunMain API and keeps normal
arguments, signals, environment, virtual environment and `sys.executable`.
Subprocesses explicitly launched later through `sys.executable` still use the
original interpreter. There is no global Python replacement or shell change.

The former `shared_python_runtime` launch argument is no longer declared.
Matching debugger prefixes are composed ahead of the native runtime launcher;
the debugger prefix filter still applies to the detector or executive script name.
No administrator rights, package download or WSL update is needed on this host;
the embedding library and development headers are already installed. Python
development headers are an explicit build dependency. An unaffected native Linux
or WSL installation whose ordinary Python/rclpy nodes already run reliably does
not need the additional launcher for application functionality. Its deployment
launch file may omit `prefix=SharedPythonPrefix(...)` while keeping the same
script and parameters. The distributed launches keep it for consistent behavior.
The recorded host is evidence for one runtime combination, not proof that all
WSL installations need a different interpreter.

## Validation

All ROS checks used private domains and simulated telemetry; none commanded
the live vehicle graph.

- Plain stock ROS worker: SIGSEGV after 45.30 seconds, same native signature.
- Stock worker pinned to CPU 0: two workers survived 120.18 seconds. Affinity
  was not applied to production because this short control is inconclusive.
- Shared-library ROS control: two workers survived separate 60.17- and
  180.17-second runs; the longer run returned 12 valid parameter replies per
  worker, no failed queries and normal exits.
- All 586 existing executive/controller/navigation/localizer/completion/monitor
  unit tests passed under the shared runtime.
- Actual executive and monitor subprocesses completed three approach/climb/left
  cycles under stock Python (79.67 seconds), three under the prototype shared
  runtime (81.27 seconds), and one under the final installed launcher (29.65
  seconds). Each cycle included a 2 m climb, a 2 m left move, survival past the
  reported left-move crash window and cancellation to armed AUTO.LOITER. There
  were no OFFBOARD watchdog losses or unintended arming/takeoff/landing/RTL.
- Runtime and launch unit checks passed: interpreter/venv identity, generated
  interface serialization, script paths and arguments, command/module modes,
  failure exit codes, SIGINT, shell quoting, opt-in defaults and process scope.
- The actual installed autonomy and monitoring launches ran together for eight
  seconds with the option enabled and no mission. Process mappings verified
  shared Python in only the executive and detection monitor; all four nodes
  and both launch parents shut down cleanly.
- Perception follow-up: both packages rebuilt successfully. All 154 existing
  perception tests passed under the shared runtime, plus eight new runtime/launch
  checks. The actual installed perception launch processed synthetic RGB and
  thermal images through OpenCV/cv_bridge, confirmed smoke and fire, published
  calibrated guidance and overlays, and cleared detections on clear frames.
  Fire confirmed after 14 processed frames and 1.594 seconds with the unchanged
  1.5-second/five-frame requirement. Both detectors executed the package-local
  native launcher with no stock-interpreter executable mapping and shut down
  cleanly. Only the test's temporary smoke calibration window was shortened to
  three frames; production calibration remains unchanged.

Passing bounded controls establishes compatibility, not full Gazebo reliability
or a confirmed repair. The stock flight-sequence control also passed.

## Historical change record

Source snapshots and native diagnostic evidence from the original investigation
are retained at:

`/home/s04fj3/kv_ws/rollback/20261027_reacquire_segfault/`

- `source-before.tar.gz`: executive source snapshot before the first shared-runtime
  change, including pre-existing uncommitted work.
- `perception/source-before.tar.gz`: perception snapshot before its shared-runtime
  follow-up, including pre-existing work.
- `pre-existing-changes.patch` and `status-before.txt`: original Git state.
- `this-change.patch` and `perception/this-change.patch`: the shared-runtime
  changes in their respective packages, including new files.
- `evidence/`: GDB reports, saved original signal context, machine-readable native
  findings, diagnostic scripts, runtime mappings, control summaries and test logs.

The new production files/settings are `src/python_shared_runtime.cpp`, its
CMake target/Python development dependency, the scoped `SharedPythonPrefix`
helper in autonomy, monitoring, standalone detection monitoring and perception
launchers. New tests cover runtime compatibility,
the reported multi-process viewpoint sequence and actual detector image processing.
README and this report document the permanent runtime. Existing Verify source
and all source-ranging/navigation changes from the previous task are preserved.

The earlier reverse patches describe the historical source state and are not
maintained as a current rollback mechanism. Do not apply them over later mission
or planning changes. The archive's parent contains `COLCON_IGNORE` so saved
source copies cannot be discovered as duplicate build packages; keep that marker
while retaining the diagnostic archives.
