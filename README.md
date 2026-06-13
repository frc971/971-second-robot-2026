# 971-second-robot-2026

Code for Team 971's 2026 robot, Mixtape.

<img width="2048" height="1152" alt="Image from iOS" src="https://github.com/user-attachments/assets/f611288a-d793-4837-8d16-6966504e1f9a" />

## Code Structure

- [frc.robot.lib.superstructure](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/lib/superstructure)

  Contains abstractions for motors, sensors, simulation support, and hardware configuration.

- [frc.robot.lib.shooter](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/lib/shooter)

  Contains definitions of physics solvers for shooting.

- [frc.robot.lib.superstructure](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/lib/superstructure)

  Contains abstractions for CANCoder and motor configurations for all subsystems. Also contains superclasses such as AngularSubsystem and LinearSubsystem which the subsystems extend.

- [frc.robot.subsystems.superstructure](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems/superstructure)

  Contains coordination logic for all subsystems, and controls shooter states. Implements abstract hardware classes for motors and subsystems with constants.

- [frc.robot.subsystems.superstructure.ShooterHandler.java](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems/superstructure/ShooterHandler.java)

  Implements physics solvers to calculate desired hood angle, turret location, and flywheel speed periodically based on the robot and target positions and the robot velocity. Logs shooter states and updates them based on the solutions. Checks if solutions satisfy hardware + software constraints.

- [frc.robot.subsystems.vision](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems/vision)

  We use [BOS](https://github.com/frc971/bos), a custom vision system, for pose estimation and localization. This folder contains classes used to publish data from this system to NetworkTables.

- [frc.robot.subsystems.Autos.java](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems/Autos.java)

  Contains auto paths that are linked together and manages autonomous selection and execution. BLine is used to visualize and construct autos and paths.
