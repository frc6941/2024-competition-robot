# FRC 6941 IronPulse 2024 Competition Robot

## Code Structure
`net.ironpulse.Constants`: All constants include PID, gear ratio, CAN ID, etc. 

`net.ironpulse.Main`: Program entrance

`net.ironpulse.Robot`: Robot instance

`net.ironpulse.RobotContainer`: Robot instance configs

`net.ironpulse.commands`: All commands (see [Command-based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html))

`net.ironpulse.subsystems`: All subsystems

`net.ironpulse.telemetries`: All telemetry (Data shown in the ShuffleBoard)

Usually, to finish a subsystem, you have to firstly config the motors in a subsystem class.
Then, implement many commands for it in order to complete some actions. A telemetry class is also needed.
A telemetry class should provide a `telemeterize()` method. It will then be passed into a `registerTelemetry()` method in the subsystem.

## Units and Positive Directions
1. Use METER as the unit of length. NO INCHES, FEET AND YARDS. NO CENTIMETERS AND MILLIMETERS. Transition will be stated explicitly.
2. Use DEGREE as the unit of angle. NO RADIANS.
3. Take COUNTER-CLOCKWISE (abbreviated as CCW) as the positive direction of rotation. NO CLOCKWISE (abbreviated as CW) unless stated explicitly.
4. Use RPM as the unit for rotational velocity.

**All these units should be wrapped by the [Units API](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html).**

## Rules
All the codes MUST be submitted through a pull request on GitHub. Directly pushing code into the repository is not allowed.

Please follow the [coding rules](https://github.com/frc6941/coding-rules). The pull request not following the rules will not be merged.
