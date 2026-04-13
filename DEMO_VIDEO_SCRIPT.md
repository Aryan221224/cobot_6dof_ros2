# DEMO VIDEO SCRIPT
## 6-DOF Teleoperated Precision Cobot | Portfolio Showcase
### Target: Articulus Surgical · Integra Robotics
### Duration: ~3:30 min | Format: Screen recording + voiceover

---

## PRE-RECORDING CHECKLIST

- [ ] RViz2 open with the cobot loaded and visible
- [ ] Camera angle: isometric, slightly elevated, robot centred
- [ ] Background: dark (RViz default dark theme looks best on camera)
- [ ] Terminal visible in lower-left corner (small, shows launch output)
- [ ] Microphone tested
- [ ] demo_motion.launch.py paused, ready to start

---

## SEGMENT 1 — INTRO (0:00 – 0:25)

**[Screen: RViz with cobot in home position, static]**

> "This is a 6 degrees-of-freedom teleoperated precision cobot — designed
> for two domains: surgical robotics and lab automation.
>
> The arm has a 800-millimetre spherical reach, 2-kilogram payload, and
> sub-millimetre repeatability at the tool centre point.
>
> It features a dual-mode end effector — a 2-finger parallel gripper for
> industrial grasping, and a motorised pipette module for precision
> liquid handling."

---

## SEGMENT 2 — ARCHITECTURE OVERVIEW (0:25 – 0:55)

**[Screen: Slowly rotate the arm in joint_state_publisher_gui — move J1 slowly]**

> "The mechanical design follows a 6-revolute serial configuration.
> Joints 1 through 3 — base rotation, shoulder lift, and elbow — use
> BLDC motors with harmonic drive gearboxes. Harmonic drives were chosen
> for their zero backlash and high reduction ratio — critical for the
> torque and precision these joints need.
>
> Joints 4 through 6 — the wrist — use compact servo actuators, giving
> us 3 full degrees of wrist freedom."

**[Slowly flex J2 and J3 while speaking]**

> "The URDF model encodes accurate inertia tensors, joint limits, damping,
> and friction coefficients for all 6 joints — making this simulation
> physically grounded, not just visual."

---

## SEGMENT 3 — FULL WORKSPACE DEMO (0:55 – 1:50)

**[Start demo_motion.launch.py — let it run automatically]**

> "Let me show the full workspace. The arm starts from home position —"

**[Pause while arm extends to full reach]**

> "— extends to full reach at 800 millimetres —"

**[Pause during J1 sweep]**

> "— sweeps 180 degrees in base rotation, covering the full work envelope —"

**[Pause during pick approach]**

> "— moves into a pick-and-place approach posture —"

**[Pause during gripper close/open]**

> "— and here you can see the parallel gripper actuating. Both fingers
> are mimic-jointed, driven by a single prismatic command."

---

## SEGMENT 4 — SURGICAL PRECISION POSTURE (1:50 – 2:20)

**[Arm moves to surgical pose — slow, deliberate]**

> "This is the surgical precision posture. The wrist has 3 independent
> axes — roll, pitch, and rotation — which allows the tool to approach
> a target from any direction without repositioning the entire arm.
>
> In a teleoperation scenario, this wrist dexterity is what enables
> non-homothetic motion scaling — where we compress large operator
> hand movements into fine micro-scale motions at the tool tip.
> This is the core principle behind surgical cobot design."

**[Show J4 and J6 rotating in opposite directions]**

> "The impedance control layer sitting above the PID loop gives the
> arm compliant behaviour — it yields when it contacts soft tissue
> rather than applying blind force."

---

## SEGMENT 5 — LAB AUTOMATION / PIPETTE (2:20 – 2:55)

**[Arm moves to pipette-down posture]**

> "Switching to lab mode. The pipette module shares the tool flange
> via a quick-swap mount — so the same arm handles both tasks.
>
> The arm moves to a vertical downward posture — ideal for well-plate
> work —"

**[Aspirate motion — plunger retracts]**

> "— and here the plunger actuates for aspiration —"

**[Dispense motion]**

> "— and dispense. Volume accuracy is controlled by the prismatic
> travel of the plunger, which can be position-controlled to micrometre
> precision."

---

## SEGMENT 6 — SOFTWARE STACK (2:55 – 3:20)

**[Screen: Switch to terminal, show ros2 topic list or rqt_graph]**

> "The software runs on ROS2 Jazzy. The robot state publisher
> broadcasts joint transforms over TF2. The joint trajectory
> controller interfaces with ros2_control — so this simulation
> is drop-in ready for real hardware by swapping the mock component
> plugin for an actual hardware interface.
>
> The full package — URDF, launch files, controller config, and this
> demo node — is on my GitHub."

---

## SEGMENT 7 — CLOSE (3:20 – 3:30)

**[Return to home position, arm centred on screen]**

> "This project demonstrates end-to-end robotics system thinking —
> from mechanical design and sensor integration through to
> control architecture and ROS2 software. Thank you for watching."

---

## RECORDING TIPS

**Pacing:** Speak at 80% of your normal speed. Pause 1 second after each
major visual change before continuing narration.

**B-roll ideas:**
- Close-up on the gripper opening/closing (zoom into RViz)
- TF frame tree displayed alongside the arm
- rqt_graph showing the node/topic architecture
- Side-by-side: arm moving + joint_states topic echo in terminal

**Thumbnail:** Home position, front-isometric view, dark background,
text overlay: "6-DOF Cobot | ROS2 Jazzy | URDF + RViz"

**Upload title suggestion:**
"6-DOF Teleoperated Cobot — Full ROS2 Simulation | Gripper + Pipette
Dual End Effector | Impedance Control | Aryan | Bennett University"

**Tags:** ros2, robotics, urdf, rviz, cobot, teleoperation, surgical
robotics, joint trajectory, ros jazzy, 6dof, robot simulation,
mechatronics, embedded systems

---

## LINKEDIN POST CAPTION

> Built a full 6-DOF teleoperated cobot in ROS2 Jazzy — URDF,
> RViz visualisation, joint trajectory control, and a dual-mode
> end effector (parallel gripper + motorised pipette).
>
> Key design decisions:
> 🔹 Harmonic drives at J1-J3 for zero-backlash precision
> 🔹 3-axis wrist for surgical non-homothetic motion scaling
> 🔹 Impedance control layer for compliant force interaction
> 🔹 ros2_control integration — hardware-ready with mock plugin
>
> Targeting applications in surgical robotics and lab automation.
>
> Full simulation code on GitHub → [link]
>
> #ROS2 #Robotics #Mechatronics #SurgicalRobotics #URDF #RViz
> #BennettUniversity #Cobot #LabAutomation
