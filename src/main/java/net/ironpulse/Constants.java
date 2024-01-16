package net.ironpulse;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.*;
import net.ironpulse.subsystems.SwerveSubsystem;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    public static class TunerConstants {
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(6);
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        // Swerve Gains
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100)
                .withKI(0)
                .withKD(0.05)
                .withKS(0)
                .withKV(1.5)
                .withKA(0);

        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(3)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final SwerveModule.ClosedLoopOutputType steerClosedLoopOutput =
                SwerveModule.ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput =
                SwerveModule.ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Measure<Current> slipCurrent = Amps.of(300.0);

        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(5.0);

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double COUPLE_RATIO = 3.5;

        private static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

        // Estimated at first, then fudge-factored to make odom match record
        private static final Measure<Distance> wheelRadius = Meters.of(0.055);

        private static final boolean STEER_MOTOR_REVERSED = true;
        private static final boolean INVERT_LEFT_SIDE = false;
        private static final boolean INVERT_RIGHT_SIDE = true;

        private static final String CAN_BUS_NAME = "rio";
        //private static final String CAN_BUS_NAME = "6941CANivore1";
        private static final int PIGEON_ID = 1;

        private static final double STEER_INERTIA = 0.00001;
        private static final double DRIVE_INERTIA = 0.001;
        private static final Measure<Voltage> steerFrictionVoltage = Volts.of(0.25);
        private static final Measure<Voltage> driveFrictionVoltage = Volts.of(0.25);

        private static final SwerveDrivetrainConstants DrivetrainConstants =
                new SwerveDrivetrainConstants().withPigeon2Id(PIGEON_ID).withCANbusName(CAN_BUS_NAME);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withWheelRadius(wheelRadius.in(Inches))
                .withSlipCurrent(slipCurrent.magnitude())
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(speedAt12Volts.magnitude())
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(steerFrictionVoltage.magnitude())
                .withDriveFrictionVoltage(driveFrictionVoltage.magnitude())
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_REVERSED);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 5;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        private static final int FRONT_LEFT_ENCODER_ID = 9;
        private static final double FRONT_LEFT_ENCODER_OFFSET = -0.000244140625;

        private static final Measure<Distance> frontLeftXPos = Meters.of(10.5); // TODO
        private static final Measure<Distance> frontLeftYPos = Meters.of(10.5); // TODO

        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
        private static final int FRONT_RIGHT_ENCODER_ID = 21;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = -0.433349609375;

        private static final Measure<Distance> frontRightXPos = Meters.of(10.5); // TODO
        private static final Measure<Distance> frontRightYPos = Meters.of(-10.5); // TODO

        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 14;
        private static final int BACK_LEFT_ENCODER_ID = 20;
        private static final double BACK_LEFT_ENCODER_OFFSET = -0.31298828125;

        private static final Measure<Distance> backLeftXPos = Meters.of(-10.5); // TODO
        private static final Measure<Distance> backLeftYPos = Meters.of(10.5); // TODO

        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
        private static final int BACK_RIGHT_ENCODER_ID = 12;
        private static final double BACK_RIGHT_ENCODER_OFFSET = 0.005615234375;

        private static final Measure<Distance> backRightXPos = Meters.of(-10.5); // TODO
        private static final Measure<Distance> backRightYPos = Meters.of(-10.5); // TODO

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET,
                frontLeftXPos.magnitude(),
                frontLeftYPos.magnitude(),
                INVERT_LEFT_SIDE);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET,
                frontRightXPos.magnitude(),
                frontRightYPos.magnitude(),
                INVERT_RIGHT_SIDE);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET,
                backLeftXPos.magnitude(),
                backLeftYPos.magnitude(),
                INVERT_LEFT_SIDE);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET,
                backRightXPos.magnitude(),
                backRightYPos.magnitude(),
                INVERT_RIGHT_SIDE);

        public static final SwerveSubsystem DriveTrain =
                new SwerveSubsystem(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }
}
