# Python node segmentation faults

The October 27 executive/detection-monitor/perception recurrence, standard-only
reproduction and same-version shared-runtime validation are recorded in
[the October 27 report](20261027_reacquire_segfault.md). The package launches now
use that runtime permanently; their earlier opt-in and rollback controls have
been removed. The underlying native fault remains unresolved.

Investigation of the October 23, 2026 incident found the same native failure
in the mission executive, battery monitor, and smoke detector. This is not
an ordinary Python exception, and the evidence does not establish a fix in
the mission state machine.

## Evidence

- The executive completed its first TAKEOFF / SWEEP / EXECUTE_PATH mission.
  The next airborne goal reached sweep step 2 before the process died.
- Launch recorded SIGSEGV exits for executive PID 84334, battery PID 84335,
  and smoke PID 84413 within about five seconds. The planner remained alive.
- WSL retained ELF cores in Windows `%LOCALAPPDATA%\Temp\wsl-crashes`.
  The relevant names contain `1792765381-84334`, `1792765382-84335`, and
  `1792765386-84413`.
- All three cores stop in Python 3.12's main thread at instruction `0x5e3259`,
  decrementing the thread state's C recursion counter. The thread state is
  valid and its target address is writable. The signal's reported fault
  address differs from the instruction's computed target by 4 TiB.
- The executive and battery Python frames are in the ROS executor's
  `_wait_for_ready_callbacks` generator, not executing mission or battery
  callbacks. This alone does not identify a bug in that executor.
- Kernel logs show the same instruction and fault-address pattern in other
  Python processes on October 22, before the battery monitor was added.
- The installed interpreter is Ubuntu Python `3.12.3-1ubuntu0.17`, running
  under WSL2 kernel `6.6.87.2-microsoft-standard-WSL2`. The configured virtual
  environment points to that system interpreter.

These observations point to a shared native/runtime problem. They do not
identify a particular ROS middleware defect or prove a WSL defect. Changing
the mission sequence, suppressing failures, or automatically restarting an
airborne flight controller would not resolve the demonstrated failure.

## Capturing another occurrence

The executive and smoke-detector Python entrypoints enable `faulthandler`
for all Python threads.
If a native crash happens again, the launch output should include Python
thread stacks. Preserve the full launch log and corresponding WSL core.
The feature also works when the executables run without a launch file.

The existing cores can be opened read-only using:

```bash
gdb /usr/bin/python3.12 /path/to/wsl-crash-...dmp
```

Useful GDB commands are `thread apply all bt`, `info registers`,
`p $_siginfo`, and `info proc mappings`. Python debugging symbols are needed
for full source-level interpreter details. Do not delete the crash dumps
before collecting that information.

The repeated-goal ROS test uses an isolated simulated FCU and simulated
`/clock`, including fresh action clients for successive goals. It exercises
the reported goal sequence without commanding a real vehicle.

If crashes recur outside the isolated test, update WSL using `wsl --update`
in Windows and test after a clean WSL restart. `wsl --shutdown` stops all WSL
processes; run it only after stopping the simulation and vehicle sessions.
This is a diagnostic environment step, not a confirmed repair for this case.

## Standalone monitor recurrence

The later standalone monitor run (PID 91953) stored a 14.93 m path with
unchanged 50% charge. That record was correctly excluded from learning.
Approximately 123 seconds later, the monitor crashed in the ROS executor;
the executive (PID 91862) crashed roughly nine seconds after that. The new
ELF cores retain the same original interpreter fault described above, beneath
the fault-handler signal frames. There is no battery calculation on the
monitor's failing stack.

No calculation divides by battery depletion. The stored rate divides
consumed percentage by travelled distance, and an empty set of usable records
produces an unavailable estimate without dividing. A Python arithmetic error
would produce an exception rather than these native fault records.

The PX4 simulator's default 50% charge floor explains the unchanged reading;
see the README's simulated battery settings for a meaningful depletion test.
This explains the measurement, not the native crashes. A combined ROS
regression runs the executive together with a separate monitor process at
constant 50% charge through repeated missions to cover this integration case.

## Monitor subscription bookkeeping defect

That integration test exposed a concrete application defect: the monitor
assigned its own list to `self._subscriptions`, a private `rclpy.Node` member.
This replaced the node's complete subscription collection with just the five
monitor subscriptions and excluded the TimeSource's `/clock` subscription.
Consequently, the ROS executor did not service clock updates, and the
monitor's periodic return-estimate timer did not run with simulated time.

The application list is now named `_monitor_subscriptions`, preserving the
node's internal collection. Both node classes were audited for other member
name collisions with the installed Jazzy `Node`; none were found.

This fixes the demonstrated timer failure. It does not yet establish the
cause of the native segmentation faults: TimeSource still retained a strong
reference to the excluded clock subscription, and the executive has its own
process and subscription collection. The later monitor core confirms this
correction was loaded, but the native failure still recurred as described below.

## Recurrence after the clock correction

The standalone monitor PID 95637 recorded two zero-depletion intervals and
then crashed between missions. Its core
`wsl-crash-1792770179-95637-_usr_bin_python3.12-11.dmp` contains the corrected
`_monitor_subscriptions` code. The original fault remains at `0x5e3259`, with
the signal's fault address 4 TiB above the instruction's writable target.
The Python stack is in executor handle polling, outside monitor callbacks.

The executive PID 95596 completed the third mission successfully. Its log
then ends roughly 49 seconds later while idle; its core
`wsl-crash-1792770385-95596-_usr_bin_python3.12-11.dmp` has the same native
fault signature. Separate processes do not show that one crash caused the
other. The shared signature warrants investigation beyond battery arithmetic.

## Native monitor backend

Production launches now run `battery_monitor_node`, implemented in C++ with
`rclcpp` and a single-threaded executor. The legacy `battery_monitor_node.py`
entrypoint replaces itself with that installed executable before loading
Python ROS libraries. Topics, message types, parameters, baseline handling,
keyed in-memory records, learning rules, safety buffers and INFO-only alerts
are preserved. The Python modules remain as a reference for regression tests.

This is a bounded workaround that removes the monitor from the repeatedly
failing CPython/rclpy execution path. It does not prove the cause of the
underlying fault or repair other Python processes. The executive remains
unchanged by this backend switch. No automatic restart was added: a restart
would clear the learned battery history and could conceal recurring failure.

The native DDS regression checks zero depletion, positive depletion, weighted
learning, safety margins, alerts, frozen home and invalid telemetry through
the public message interfaces. The combined action regression completes
three full missions with unchanged 50% charge, then cancels a fourth mission
at sweep step four, verifying records and estimates across process boundaries.

### Validation on October 23

- The package builds successfully. The 62 Python reference unit tests, native
  DDS regression and combined repeated-mission regression pass. The combined
  regression's final run completed in 63.24 seconds without shutdown warnings.
- The installed standalone launch starts the native executable with its YAML
  parameters and shuts down cleanly on SIGINT. C++ formatting, Python style
  checks for the changed launchers/tests, and CMake lint pass.
- Separate eight-minute runs of the frozen Python reference and installed
  native monitor both survived four scanning/path intervals. Each received
  48,000 odometry publications, 24,000 clock updates and 2,400 battery updates.
  Each published four usage records and 340 estimates, including 94 estimates
  with an active alert. The first three paths consumed zero measured charge
  and were excluded; the last travelled approximately 15.04 m and consumed
  20 percentage points, producing a usable sample and INFO return alerts.
- Run artifacts are retained in
  `/tmp/kv_battery_python_soak_20261023/` and
  `/tmp/kv_battery_native_soak_20261023/`, including `summary.json`,
  `driver.log`, `worker.log` and the isolated driver script.

These isolated tests did not reproduce the Python SIGSEGV. They validate
the native implementation's sustained operation and battery calculations,
but do not establish the runtime fault's cause or replace testing in the
original SITL environment.

## Executive recurrence and independent driver reproduction

The later executive PID 103855 failed approximately 34 seconds after startup.
Its mission worker had completed takeoff and was waiting in
`_sweep -> _enter_offboard -> _wait_until`. The main thread again stopped at
the CPython recursion-counter store in executor polling. Recent mission
changes only adjust scan/path timings; the subscriptions, clients, action
server, TF listener and executor setup match the original Git version. The
executive imports no battery modules, and the monitor is disabled by default
in the autonomy launch.

An isolated telemetry driver then reproduced the identical fault after about
110 seconds (PID 104851). That driver imports only the standard library,
`rclpy` and standard ROS/MAVROS message/service packages. It creates a plain
ROS node, publishes telemetry and queries parameters; it imports no
`kv_executive` modules or custom interfaces and sends no mission goals.
Two separate installed executives were receiving its isolated telemetry.

Unlike the earlier processes, this driver had no `faulthandler` installed,
so its core retains the original kernel signal directly:

```text
SIGSEGV, si_code=SEGV_MAPERR
RIP = 0x5e3259: 89 47 24  (mov %eax,0x24(%rdi))
RDI = 0xba6748
Computed store destination = 0xba676c
Reported fault destination = 0x40000ba676c  (+4 TiB)
```

Its only Python thread was executing
`contextlib._push_cm_exit -> enter_context -> rclpy._wait_for_ready_callbacks
-> spin_once`. The target thread-state memory is valid and writable, and its
recursion counter is normal (9996 before the attempted decrement). This
independently confirms the earlier signal-frame interpretation and shows
that mission and battery application logic are unnecessary to reproduce
this native signature. It still does not establish the responsible component.

Raw ELF verification rules out GDB substituting executable-file contents:
the dumped executable segment contains the actual instruction bytes, and
its entire 4 KiB page matches the installed Python binary. A fully dumped RW
segment contains the thread-state destination and counter; the reported
fault destination lies in no dumped address mapping.

The driver and output are retained in `/tmp/kv_executive_idle_104851/`; its
core is `wsl-crash-1792776552-104851-_usr_bin_python3.12-11.dmp`.
Read-only GDB reports are `/tmp/driver104851_initial.txt` and
`/tmp/driver104851_frames.txt`. The executive core is
`wsl-crash-1792776177-103855-_usr_bin_python3.12-11.dmp`.

As a separate control, the actual installed executive ran in its own process
with an isolated simulated FCU: three full goals (takeoff, then two airborne
goals) and another 40 seconds idle completed without SIGSEGV. The full run
took 84.29 seconds, made three planning requests, and had no OFFBOARD watchdog
losses. Results are in `/tmp/kv_executive_process_actions_105412/`.

Package verification found no modified files in the checked Python and ROS
runtime packages. The available `rclpy` 7.1.12 update changes worker-pool
joining during shutdown; it leaves polling unchanged, so it is not an
established repair for these live-process failures. The relevant TF Python
and native sources are unchanged between installed 0.36.21 and available
0.36.23. Upstream comparisons:
[rclpy shutdown patch](https://github.com/ros2/rclpy/pull/1670) and
[rclpy changelog](https://github.com/ros2/rclpy/blob/7.1.12/rclpy/CHANGELOG.rst).

The checked host runtime is WSL 2.6.3.0 with kernel 6.6.87.2-1, Windows
10.0.26200.9711, on an Intel Core Ultra 7 265HX. No matching upstream issue
or confirmed repair was found for the 4 TiB address discrepancy. A plain
stdlib `ExitStack` control completed over 15 million iterations in 30 seconds
without ROS; that passing short control is inconclusive about the native fault.

### ROS-only probe

`scripts/diagnose_ros_executor.py` replaces both executive children with plain
`rclpy` workers. Parent and children import only standard ROS messages and
services; no process imports MAVROS, YASMIN, actions, `kv_executive` modules or
custom interfaces. The probe sends synthetic odometry, clock, TF and Boolean
state to its own workers and checks their standard parameter services. All
ROS names use a unique namespace and a separate domain. Run it without
launching simulation or autonomy:

```bash
source /home/s04fj3/kv_ws/install/setup.bash
ros2 run kv_executive diagnose_ros_executor.py --duration 180
```

The first 10-second run reproduced SIGSEGV in both workers (PIDs 106333 and
106334), after each had answered its parameter query correctly. Artifacts,
including a frozen source copy, are in
`/tmp/kv_executor_probe_smoke_20261023/`. Both cores have the same 4 TiB
address discrepancy, but different interpreter stores:

| PID | Instruction destination | Reported fault address |
| --- | --- | --- |
| 106333 | `0xba676c` (recursion counter) | `0x40000ba676c` |
| 106334 | `0x7a6ca91ad4b8` (interpreter frame) | `0x7e6ca91ad4b8` |

Raw core bytes confirm the second instruction and its valid writable target
as well. This extends the evidence beyond the recursion-counter instruction
and reproduces the signature without either production application process.

### Middleware controls

A temporary CycloneDDS overlay was extracted under
`/tmp/kv_ros_cyclone_control/` from authenticated APT packages; no system
package installation or production default change was made. Its
`packages.txt` records versions and hashes. Under that overlay:

- The original paired executive idle driver completed 180 seconds with both
  executives responsive and exiting normally (12 valid parameter replies each).
  Results: `/tmp/kv_executive_idle_106079/result.json`.
- The exact ROS-only probe completed 180 seconds with both workers exiting
  normally and 12 valid parameter replies each. Results:
  `/tmp/kv_executor_probe_cyclone_20261023/summary.json`.
- The actual executive completed three goals followed by 40 seconds idle,
  with 812 setpoints, no OFFBOARD watchdog loss and a normal exit. Results:
  `/tmp/kv_executive_process_actions_106810/result.json`.
- A FastDDS action client discovered a CycloneDDS executive, but its goal
  acknowledgement timed out. This mixed-backend check failed without a
  native crash. Results: `/tmp/kv_executive_process_actions_107379/result.json`.
  Switching just the executive's middleware has therefore not passed
  interoperability testing; a CycloneDDS control must use that backend
  consistently across the communicating processes and ROS action CLI.

These passing controls are candidates for further testing, not a diagnosis
of FastDDS as the source of the inconsistent address. The earlier Python
monitor also survived an eight-minute FastDDS control, demonstrating that
a short successful run alone cannot establish a repair.

### Executive UDP-only workaround

Fast DDS was also tested with its shared-memory transport disabled, preserving
the existing RMW implementation. A 180-second ROS-only probe completed with
both workers exiting normally and 12 valid parameter replies each. Process
maps confirmed no Fast DDS shared-memory mappings. Artifacts are in
`/tmp/kv_fastdds_udp_executor_20261023/run_corrected/`; the workers emitted
nonfatal `Destroyable` warnings during shutdown. The earlier default-transport
probe had crashed in both workers in under ten seconds.

A separate interoperability check used an actual installed executive with
UDP only, while the simulated FCU and action client kept default Fast DDS
transport (including shared memory). Takeoff, sweep, planned flight and
five seconds idle completed with no OFFBOARD watchdog loss or SIGSEGV.
The executive exited normally after 22.44 seconds. Artifacts are in
`/tmp/kv_executive_process_actions_107667/`. This check passed where the
mixed-RMW action check failed.

`autonomy.launch.py` now enables `executive_udp_only` by default. A scoped
launch group selects `config/fastdds_udp.xml` for just the executive when
using default/static Fast DDS. Planner, MAVROS and monitor processes keep
their inherited environments. The profile changes participant transport;
publisher/subscriber QoS, mission actions and flight timings are unchanged.
No additional middleware installation is required.

The installed production profile then passed three full missions followed
by 40 seconds with live simulated telemetry (38 fresh idle heartbeats), with
the executive exiting normally after 84.07 seconds. The default-transport FCU
and action clients completed three planning calls and one arm request. This
native-crash/lifetime test used a 2-second simulated OFFBOARD watchdog; its
observed setpoint intervals averaged 0.100 seconds with a maximum of 0.499
seconds despite requesting 20 Hz. It does not certify a sustained 20 Hz rate
or a strict 500 ms control margin. Results, driver and tested XML snapshots:
`/tmp/kv_executive_process_actions_108377/`.

A final targeted timing check kept the simulated OFFBOARD watchdog at a
strict 0.5 seconds. One full mission plus five seconds live-telemetry idle
passed in 24.94 seconds, with no mode abort or watchdog loss and a maximum
observed setpoint gap of 0.402 seconds. This remains an isolated simulated-clock
check, not a sustained-rate certification. Artifacts:
`/tmp/kv_executive_process_actions_108933/`. The driver emitted a nonfatal
`Destroyable` warning during its final teardown after reporting the result.

An actual installed autonomy launch also passed a 12.18-second isolated wiring
check with legacy localhost mode enabled. Process environments and maps
confirmed UDP/no SHM and the scoped localhost override only in the executive;
the planner retained its original environment and SHM. Both exited normally.
Evidence: `/tmp/kv_autonomy_scoped_transport_108734/summary.json` and
`snapshots.json`. Separate launch-helper checks cover 20 configuration and
scope-restoration cases.

The installed RMW's `LOCALHOST` discovery branch appends shared memory even
when a custom XML profile disables builtin transports. The launch therefore
uses `fastdds_udp_localhost.xml` to preserve an explicitly local network scope
and overrides that branch inside the executive only. It also handles the
deprecated `ROS_LOCALHOST_ONLY=1`, which takes precedence over the newer
range setting in the installed RCL. Other discovery ranges, static-peer
settings and selected RMW implementations are preserved. Source references:
[RMW transport setup](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp)
and [RCL initialization](https://github.com/ros2/rcl/blob/9.2.11/rcl/src/rcl/init.c).

To preserve an inherited custom Fast DDS profile or discovery-server setup:

```bash
ros2 launch kv_executive autonomy.launch.py executive_udp_only:=false
```

This is a reversible transport workaround supported by bounded controls.
It does not prove a Fast DDS shared-memory defect or explain the inconsistent
addresses in the cores. Continued SITL testing is needed to establish its
effectiveness in the original environment.

## Perception and detection-monitor recurrence on October 25

Before starting autonomy, the user observed two thermal-detector failures and
one detection-monitor failure. The exact launch records and corresponding WSL
cores were examined read-only:

| Process | PID | Launch start, Unix seconds | Death logged, Unix seconds | Approximate runtime |
| --- | --- | --- | --- | --- |
| Thermal fire detector | 25758 | 1792928817.514 | 1792928853.972 | 36.46 s |
| Detection monitor | 25834 | 1792928834.478 | 1792928885.661 | 51.18 s |
| Restarted thermal fire detector | 25904 | 1792928877.508 | 1792928894.472 | 16.96 s |

The relevant launch directories under `~/.ros/log/` are
`2026-10-25-11-46-57-451126-MD-063906-25754`,
`2026-10-25-11-47-14-403116-MD-063906-25830`, and
`2026-10-25-11-47-57-440153-MD-063906-25900`. The monitor's death was logged
approximately eight seconds after the second perception launch. That ordering
does not establish that either process caused the other's failure. The native
battery monitor remained alive until the user stopped its launch; both smoke
detectors shut down cleanly on SIGINT.

The cores under
`/mnt/c/Users/s04fj3/AppData/Local/Temp/wsl-crashes/` are:

- `wsl-crash-1792928852-25758-_usr_bin_python3.12-11.dmp`
- `wsl-crash-1792928884-25834-_usr_bin_python3.12-11.dmp`
- `wsl-crash-1792928893-25904-_usr_bin_python3.12-11.dmp`

All three retain the original `SIGSEGV`, `SEGV_MAPERR` signal at `0x5e3259`.
The instruction `mov %eax,0x24(%rdi)` has `RDI = 0xba6748`, giving an expected
store destination of `0xba676c` inside a writable interpreter mapping. The
reported fault address is again `0x40000ba676c`, exactly 4 TiB higher. The
stored recursion counter is 9996 and the attempted new value is 9995. This
matches the earlier executive, monitor, smoke-detector and plain ROS-probe
signature. All three cores contain Fast DDS shared-memory participant and
port mappings.

Both fire-detector Python stacks are in
`contextlib.enter_context -> rclpy._wait_for_ready_callbacks`. The detection
monitor is also in `_wait_for_ready_callbacks`. These failing main-thread
stacks contain no image-processing or detection-monitor callback. Python
frames were read directly from core memory using offsets verified against
the installed CPython headers; no inferior function was called. Unlike the
older raw-code-page check described above, these newer cores omit executable
pages: GDB obtains the instruction bytes from the unchanged installed Python
binary. The writable interpreter-state mapping is retained in the cores.
Reports, frame-decoding commands and the audit are retained in
`/tmp/kv_native_fault_audit_20261025/`, including `findings.md`,
`core0.txt`, `core1.txt`, `core2.txt`, `fire0_frames.txt`,
`monitor_frames.txt`, and `fire1_frames.txt`.

The executable entrypoints use `#!/usr/bin/env python3`; the cores identify
`/usr/bin/python3.12`. The configured virtual environment points to the same
Ubuntu Python 3.12.3 interpreter and includes system site packages. An ABI
spot check imported system OpenCV 4.6.0, NumPy 1.26.4 and ROS Jazzy
`cv_bridge` 4.1.0 and successfully converted a `mono8` image. The native
extensions have matching CPython 3.12 tags. Package verification reported no
modified files in the checked Python, OpenCV, `cv_bridge`, `rclpy` and
`rmw_fastrtps_cpp` packages. The detection monitor maps neither OpenCV nor
`cv_bridge`. These checks support the shared-runtime interpretation; they do
not prove the absence of every possible ABI or native-library defect.

The earlier UDP-only workaround was scoped only to the executive, so it did
not protect either perception process or the separately launched detection
monitor. `perception.launch.py` now applies that workaround to its smoke and
fire detector subprocesses by default, and the detection-monitor launch
applies it to its own Python subprocess. The scopes preserve the inherited
environments of unrelated processes, including the native battery monitor.
Both launches also enable Python fault-handler output. The respective
`perception_udp_only:=false` and `detection_monitor_udp_only:=false` arguments
preserve inherited transport and discovery settings when needed.

This extension uses the same reversible workaround supported by the earlier
bounded controls. The matching cores do not establish Fast DDS shared memory
or WSL as the root cause, and changing MAVROS launch files does not address
the demonstrated interpreter fault. No automatic process restart was added.

### Installed perception and monitoring launch validation

A private loopback ROS graph (domain 132) ran the installed
`monitoring.launch.py` continuously while `perception.launch.py` was started
three times for 60 seconds each. Total driver runtime was 180.41 seconds.
Synthetic 640 x 480 RGB and thermal images, calibration and simulation clock
were published at 20 Hz. The thermal image included broad white sky/ground
strips and a persistent compact white target. No autonomy node, real action
goal or vehicle command was used.

| Perception start | Fire messages / overlays | Smoke messages / overlays | Confirmed fire messages | Typed Abort requests |
| --- | --- | --- | --- | --- |
| 1 | 319 / 319 | 200 / 200 | 257 | 1 |
| 2 | 347 / 347 | 207 / 207 | 307 | 1 |
| 3 | 340 / 340 | 189 / 189 | 290 | 1 |

Each stage first left the monitor idle for 30 seconds, then supplied synthetic
executing-goal status and an Exploration phase. The confirmed fire reached
the monitor and produced a goal-scoped `Abort` request on every stage. This
checks request delivery, not actual flight or LOITER recovery. All processes
remained alive during their observation intervals, with no native crash.

Process environment and mapping snapshots verified that both detectors and
the Python detection monitor selected the loopback UDP profile and had no
Fast DDS shared-memory mappings. The native battery monitor retained its
inherited LOCALHOST discovery and default transport. Unrelated ROS tracing
shared-memory mappings were excluded from the Fast DDS check. The driver's
group-wide SIGINT at deliberate shutdown also reached children through the
launcher's signal forwarding; Python children then logged KeyboardInterrupt
and exit -2 during cleanup. Those intentional-stop records are distinct from
the reported spontaneous exit -11 failures.

A separate eight-second shutdown check in private domain 189 signaled only
the launch parent PIDs. Both launchers returned 0, and all four children
(smoke, fire, battery and detection monitor) finished cleanly without Python
tracebacks. Its logs and summary are in `/tmp/kv_launch_shutdown_29559/`.

The driver, summary, environment/mapping snapshots and complete launch logs
are retained in `/tmp/kv_perception_monitor_transport_28182/` as `driver.py`,
`summary.json`, `process_snapshots.json`, `monitoring.log` and
`perception_0.log` through `perception_2.log`.

Both packages rebuilt successfully. Targeted tests passed: 154 perception
tests, 32 executive launch-transport checks and 47 detection-monitor tests.
Installed `--show-args` checks passed for perception, its legacy smoke launch,
monitoring, standalone detection monitoring, autonomy, PX4 and the simulator
world launch. The launches consistently default the drone instance name to
`kv_drone`; the simulator model type remains `x500_depth`.

This is bounded synthetic evidence for the workaround, not proof of the root
cause or validation of long-duration operation with the actual Gazebo cameras.

### Recurrence in the full simulator graph

Later that day, the user reproduced the failure while running the full
`kv_world.launch.py` and perception launch. In the simulator launch log
`2026-10-25-12-51-22-173418-MD-063906-33670`, the self-filter process 33677
published filtered point clouds and then crashed after about 66 seconds.
Its respawned process 34622 published clouds and crashed about 32 seconds
later. A third process started and remained alive until the launch was
stopped. In perception launch
`2026-10-25-12-52-17-355159-MD-063906-34516`, fire detector 34520 crashed
after about 49 seconds; the smoke detector shut down cleanly when the launch
was stopped.

The exact cores are
`wsl-crash-1792932748-33677-_usr_bin_python3.12-11.dmp`,
`wsl-crash-1792932783-34622-_usr_bin_python3.12-11.dmp`, and
`wsl-crash-1792932785-34520-_usr_bin_python3.12-11.dmp`. The fire-detector
core retains the same failing CPython instruction at `0x5e3259`, with the
same 4 TiB discrepancy between the writable interpreter address and the
reported fault address. Its environment contains the configured
`fastdds_udp.xml` profile and `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, while
its core still contains Fast DDS shared-memory mappings. The earlier 180
second synthetic probe used an isolated loopback graph, so it did not
represent this full SUBNET graph. That probe was insufficient evidence that
the workaround fixed the user's launch configuration.

The self-filter's automatic restart is now capped at two retries to prevent
an unbounded cycle of process restarts and core dumps. This limits repeated
failures; it does not fix the underlying crash. The current WSL kernel
reported by `uname -r` is `6.6.87.2-microsoft-standard-WSL2` (built June 5,
2025). Since the same interpreter fault has recurred across unrelated ROS
Python nodes and transport profiles, update WSL from Windows with
`wsl --update`, then fully stop the WSL instance with `wsl --shutdown` before
retesting. This environment step is not a confirmed repair; the new kernel
version and resulting launch behavior need to be recorded after the update.

### Recurrence with perception UDP profile and no monitor

On October 25 the user ran the simulator and perception, but did not start
monitoring. The simulator self-filter (PID 38124) started at 1792933808.445
and crashed at 1792933930.785, about 122 seconds later. Launch restarted it
as PID 39099; that process published a filtered cloud and stayed up until
the user stopped the simulator. In the separate perception launch, smoke
detector PID 39007 ran for about 126.6 seconds and fire detector PID 39008
ran for about 213.1 seconds before each exited with SIGSEGV. The launch log
contains no monitoring process.

The launch records are under
`~/.ros/log/2026-10-25-13-10-08-209929-MD-063906-38117/launch.log` and
`~/.ros/log/2026-10-25-13-11-00-317365-MD-063906-39004/launch.log`. Their
cores are `wsl-crash-1792933930-38124-_usr_bin_python3.12-11.dmp`,
`wsl-crash-1792933985-39007-_usr_bin_python3.12-11.dmp`, and
`wsl-crash-1792934072-39008-_usr_bin_python3.12-11.dmp` in Windows' WSL
crash directory.

The smoke and fire cores both stop at CPython instruction `0x5e3259`
(`mov %eax,0x24(%rdi)`) with `RDI=0xba6748` and `EAX=9995`; the computed
store target is `0xba676c`. Their Python stacks are in the executor wait
path. This confirms the same faulting instruction and operands as the
previous detector incidents. In these newer cores, however, the visible
`NT_SIGINFO` is `SI_TKILL` from the fatal-signal re-raise, and the kernel log
does not record the original fault address. Therefore these cores alone do
not confirm the earlier reported 4 TiB `si_addr` discrepancy.

Both detector cores retain
`FASTRTPS_DEFAULT_PROFILES_FILE=/home/s04fj3/kv_ws/install/kv_perception/share/kv_perception/config/fastdds_udp.xml`,
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, and `ROS_DOMAIN_ID=0`. Their core
memory maps contain no `/dev/shm/fastrtps_*` mappings. This run therefore
confirms that the configured UDP-only profile was active, yet the two Python
detectors still crashed while monitoring was absent. This occurrence did not
require a monitor or detector-local Fast DDS shared-memory mappings. Other
participants kept their own transport settings, so this does not rule out
all graph-level DDS effects. The shared Python/WSL runtime remains a
candidate, not a confirmed root cause.

The self-filter core has a different interrupted CPython instruction:
`0x5d76cb` in `_PyEval_EvalFrameDefault`, rather than the detector cores'
`0x5e3259`. Its core contains numerous Fast DDS `/dev/shm/fastrtps_*`
mappings and has no UDP-profile variable in its environment. The launch
stack still shows the executor wait path, but the core's visible signal info
is also the fatal-signal re-raise; the original memory-fault address is not
available from that note. This establishes a second Python SIGSEGV
signature in the simulator process, not that it has the same immediate
fault as the detectors.

The current WSL kernel remains
`6.6.87.2-microsoft-standard-WSL2` (June 5, 2025 build). The new observation
does not identify a code-level fix or confirm WSL as the cause. After
stopping the simulation, update WSL from Windows PowerShell with
`wsl --update`, then run `wsl --shutdown` and record the new kernel version
before repeating the same launch. No ROS processes were started for this
core and log review.

## CycloneDDS alternative when WSL updates need approval

The October 26 workaround prepares CycloneDDS under
`/home/s04fj3/kv_ws/ext_install/cyclonedds`. It uses official ROS Jazzy
packages downloaded through the configured authenticated ROS APT repository
and extracted into this workspace with `dpkg-deb --extract`. No Windows
administrator rights, Linux `sudo`, system package installation, or package
rebuild is required. Versions, downloaded archives, authenticated metadata
and hashes are retained there as `packages.lock`, `debs/`, `apt-packages.txt`
and `SHA256SUMS`. The extracted packages retain their license files.

Stop the current ROS launches cleanly, then run the following in **every ROS
terminal or tmux pane**, including those used for action/service/topic CLI
commands:

```bash
source /home/s04fj3/kv_ws/install/setup.bash
source /home/s04fj3/kv_ws/ext_install/cyclonedds/setup.bash
```

Run `ros2 daemon stop` once from a prepared terminal before restarting the
usual launches. Use the helper for the simulator's ROS processes, MAVROS,
perception, monitoring, autonomy, RViz and ROS CLI clients. Keep the existing
domain, discovery settings and `drone_name:=kv_drone`. Running processes
cannot change middleware after startup. Existing tmux panes retain their
environment, and a previously running tmux server may supply an old
environment even to a new session; source the helper inside each ROS pane.

The helper selects `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and supplies the
local ament prefix and both library directories. The existing executive and
perception launch helpers already honor a different selected RMW, so they
do not apply their Fast DDS XML profiles in this configuration. CycloneDDS
shared memory is disabled by default; no RouDi daemon is needed. An inherited
`CYCLONEDDS_URI` is preserved. No shell startup file is changed. To return to
the previous middleware, stop the ROS launches and daemon, then use fresh
terminals without sourcing the helper.

Preparation checks verified every archive hash against the authenticated APT
metadata. The Bash helper passed `bash -n` and ShellCheck. Using its exact
library paths, `ldd -r` on the RMW library found no missing dependencies or
unresolved symbols; all extracted dependency libraries also resolved. These
checks initialized no ROS participant and started no ROS launches or tests.

The earlier middleware controls in this document support trying this
alternative: two 180-second runs and a three-goal simulated executive check
passed under CycloneDDS. They do not establish a repair for the full Gazebo
graph. The earlier mixed Fast DDS/CycloneDDS action check timed out, so the
selection must be consistent across this graph and its action clients.

### Fire detector recurrence with CycloneDDS active (October 26)

The user then ran only `perception.launch.py` in
`~/.ros/log/2026-10-26-09-01-37-800936-MD-063906-44366/`. Fire detector
PID 44370 started; smoke detector PID 44369 remained alive until the user
stopped the launch. Monitoring and autonomy were not started. The fire
detector core is
`/mnt/c/Users/s04fj3/AppData/Local/Temp/wsl-crashes/wsl-crash-1793005324-44370-_usr_bin_python3.12-11.dmp`.

The core maps the workspace-local `librmw_cyclonedds_cpp.so` and `libddsc.so`,
so the CycloneDDS helper was active for this detector. The core's visible
signal information is the Python fatal-handler re-raise (`SI_TKILL`). GDB
recovers the interrupted instruction at `0x5e3259` in
`_PyEval_EvalFrameDefault`: it attempts to decrement the recursion counter
from 9996 to 9995 through `RDI=0xba6748`, inside CPython's `_PyRuntime`
mapping. The stack is in the executor wait path, not in the fire-detection
callback. The kernel log still reports
`6.6.87.2-microsoft-standard-WSL2`. This establishes that switching this
process to CycloneDDS did not prevent the Python SIGSEGV; it does not establish
whether the cause is WSL, CPython, or native memory corruption elsewhere.

Microsoft documents a supported custom-kernel selection in the per-user
`%UserProfile%\\.wslconfig` file using the `kernel` setting. This changes the
kernel used by all WSL 2 distributions and takes effect after WSL stops. A
managed-device policy can disable custom-kernel configuration, so this option
must follow the organization's policy. See Microsoft's
[WSL configuration guide](https://learn.microsoft.com/windows/wsl/wsl-config)
and [WSL kernel repository](https://github.com/microsoft/WSL2-Linux-Kernel).
