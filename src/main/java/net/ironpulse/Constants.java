package net.ironpulse;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;
import net.ironpulse.subsystems.SwerveSubsystem;

public final class Constants {
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class SwerveConstants {
        // The max speed of the swerve (should not larger than speedAt12Volts)
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(6);
        // The max turning speed of the swerve
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(0.5 * Math.PI);

        // Swerve steering gains
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100)
                .withKI(0.2)
                .withKD(0.005)
                .withKS(0)
                .withKV(0)
                .withKA(0);

        // Swerve driving gains
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(2)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0);

        // The swerve heading (used in SpeakerAimingCommand) gains
        public static final Slot0Configs headingGains = new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        // The swerve gains when adjusting tx (used in AmpAimingCommand)
        public static final Slot0Configs txGains = new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final SwerveModule.ClosedLoopOutputType steerClosedLoopOutput =
                SwerveModule.ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput =
                SwerveModule.ClosedLoopOutputType.Voltage;

        // The stator current at which the wheels start to slip
        private static final Measure<Current> slipCurrent = Amps.of(300.0);

        // Theoretical free speed (m/s) at 12v applied output;
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(5.0);

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        private static final double COUPLE_RATIO = 3.5;

        private static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

        // Estimated at first, then fudge-factored to make odom match record
        private static final Measure<Distance> wheelRadius = Meters.of(0.05);

        private static final boolean STEER_MOTOR_REVERSED = true;

        // private static final String CAN_BUS_NAME = "rio";
        private static final String CAN_BUS_NAME = "6941CANivore1";
        private static final int PIGEON_ID = 1;

        // Simulation only
        private static final double STEER_INERTIA = 0.00001;
        private static final double DRIVE_INERTIA = 0.001;
        private static final Measure<Voltage> steerFrictionVoltage = Volts.of(0.25);
        private static final Measure<Voltage> driveFrictionVoltage = Volts.of(0.25);

        private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(PIGEON_ID)
                .withCANbusName(CAN_BUS_NAME);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withWheelRadius(wheelRadius.in(Meters))
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
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.SyncCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_REVERSED);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 5;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        private static final int FRONT_LEFT_ENCODER_ID = 9;
        private static final double FRONT_LEFT_ENCODER_OFFSET = 0.179688;
        // private static final double FRONT_LEFT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> frontLeftXPos = Meters.of(0.5);
        private static final Measure<Distance> frontLeftYPos = Meters.of(0.5);

        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
        private static final int FRONT_RIGHT_ENCODER_ID = 21;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.376709;
        // private static final double FRONT_RIGHT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> frontRightXPos = Meters.of(0.5);
        private static final Measure<Distance> frontRightYPos = Meters.of(-0.5);

        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 14;
        private static final int BACK_LEFT_ENCODER_ID = 20;
        private static final double BACK_LEFT_ENCODER_OFFSET = 0.167969;
        // private static final double BACK_LEFT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> backLeftXPos = Meters.of(-0.5);
        private static final Measure<Distance> backLeftYPos = Meters.of(0.5);

        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
        private static final int BACK_RIGHT_ENCODER_ID = 12;
        private static final double BACK_RIGHT_ENCODER_OFFSET = 0.431641;
        // private static final double BACK_RIGHT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> backRightXPos = Meters.of(-0.5);
        private static final Measure<Distance> backRightYPos = Meters.of(-0.5);

        private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET,
                frontLeftXPos.magnitude(),
                frontLeftYPos.magnitude(),
                false);
        private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET,
                frontRightXPos.magnitude(),
                frontRightYPos.magnitude(),
                true);
        private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET,
                backLeftXPos.magnitude(),
                backLeftYPos.magnitude(),
                false);
        private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET,
                backRightXPos.magnitude(),
                backRightYPos.magnitude(),
                true);

        public static final SwerveSubsystem DriveTrain =
                new SwerveSubsystem(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 40;

        public static final Measure<Voltage> indexVoltage = Volts.of(7);
    }

    public static class ShooterConstants {
        public static final int SHOOTER_L_MOTOR_ID = 41;
        public static final int SHOOTER_R_MOTOR_ID = 42;
        public static final int ARM_MOTOR_ID = 0;

        // Shooter gains when deploying shooter to desired angle
        public static final Slot0Configs armGains = new Slot0Configs()
                .withKP(0.1)
                .withKI(0)
                .withKD(0.1)
                .withKV(0.12)
                .withKS(0.25);

        public static final Measure<Current> armZeroCurrent = Amps.of(12);
        public static final Measure<Voltage> armZeroVoltage = Volts.of(1);

        public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(10)
                .withMotionMagicJerk(50)
                .withMotionMagicCruiseVelocity(5);
        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(1);

        public static final Measure<Voltage> shootVoltage = Volts.of(16);

        public static final Measure<Angle> ampDeployAngle = Degrees.of(0);

        public static final Measure<Voltage> manualAimingVoltage = Volts.of(0);
    }

    public static class IntakerConstants {
        public static final int INTAKER_MOTOR_ID = 30;

        public static final Measure<Voltage> intakeVoltage = Volts.of(1);
    }

    public static class BeamBreakConstants {
        public static final int INTAKER_BEAM_BREAK_ID = 30;
        public static final int INDEXER_BEAM_BREAK_ID = 40;
        public static final int SHOOTER_BEAM_BREAK_ID = 41;
        public static final int SHOOTER_R_BEAM_BREAK_ID = 42;
    }

    public static class IndicatorConstants {
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 0;
    }
}
