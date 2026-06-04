# 971-second-robot-2026

Code for Team 971's 2026 robot, Mixtape.

<img width="2048" height="1152" alt="Image from iOS" src="https://github.com/user-attachments/assets/f611288a-d793-4837-8d16-6966504e1f9a" />

## Code Structure

- [frc.robot.lib.superstructure](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/lib/superstructure)

  Contains abstractions for motors, sensors, simulation support, and hardware configuration.

- [frc.robot.lib.shooter](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/lib/shooter)

  Contains physics solvers for shooting

- [frc.robot.subsystems](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems)

  Contains all subsystems

- [frc.robot.subsystems.superstructure](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems/superstructure)

  Contains coordination logic for all subsystems, and controls shooter states.

- [frc.robot.subsystems.Autos.java](https://github.com/frc971/971-second-robot-2026/tree/main/src/main/java/frc/robot/subsystems/Autos.java)

  Contains auto paths that are linked together and manages autonomous selection and execution.
