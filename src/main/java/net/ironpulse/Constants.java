package net.ironpulse;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.*;
import net.ironpulse.subsystems.swerve.SwerveSubsystem;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final Mode currentMode = Mode.REAL;

    public static String CAN_BUS_NAME = "6941CANivore1";

    public enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static class SwerveConstants {
        // The max speed of the swerve (should not larger than speedAt12Volts)
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(6);
        // The max turning speed of the swerve
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        public static final SlewRateLimiter xLimiter =
                new SlewRateLimiter(3, -3.25, 0);
        public static final SlewRateLimiter yLimiter =
                new SlewRateLimiter(3, -3.25, 0);

        // Swerve steering gains
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(120)
                .withKI(0.2)
                .withKD(0.005)
                .withKS(0)
                .withKV(0)
                .withKA(0);

        // Swerve driving gains
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(0.01)
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

        // The stator current at which the wheels start to slip
        private static final Measure<Current> slipCurrent = Amps.of(300.0);

        // Theoretical free speed (m/s) at 12v applied output;
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(6.0);

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        private static final double COUPLE_RATIO = 3.5;

        private static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

        // Estimated at first, then fudge-factored to make odom match record
        private static final Measure<Distance> wheelRadius = Meters.of(0.05);

        private static final boolean STEER_MOTOR_REVERSED = true;

        // private static final String CAN_BUS_NAME = "rio";

        private static final int PIGEON_ID = 1;
        public static final Measure<Voltage> climbDriveVoltage = Volts.of(2);

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
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.SyncCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_REVERSED);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 5;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        private static final int FRONT_LEFT_ENCODER_ID = 9;
        private static final double FRONT_LEFT_ENCODER_OFFSET = 0.153809;
        // private static final double FRONT_LEFT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> frontLeftXPos = Meters.of(0.5);
        private static final Measure<Distance> frontLeftYPos = Meters.of(0.5);

        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
        private static final int FRONT_RIGHT_ENCODER_ID = 21;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.381592;
        // private static final double FRONT_RIGHT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> frontRightXPos = Meters.of(0.5);
        private static final Measure<Distance> frontRightYPos = Meters.of(-0.5);

        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 14;
        private static final int BACK_LEFT_ENCODER_ID = 20;
        private static final double BACK_LEFT_ENCODER_OFFSET = 0.162598;
        // private static final double BACK_LEFT_ENCODER_OFFSET = 0;

        private static final Measure<Distance> backLeftXPos = Meters.of(-0.5);
        private static final Measure<Distance> backLeftYPos = Meters.of(0.5);

        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
        private static final int BACK_RIGHT_ENCODER_ID = 12;
        private static final double BACK_RIGHT_ENCODER_OFFSET = 0.430176;
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
        public static final Slot0Configs headingGains = new Slot0Configs()
                .withKP(0.04)
                .withKI(0)
                .withKD(0);

        public static final SwerveSubsystem DriveTrain =
                new SwerveSubsystem(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
    }

    public static class IndexerConstants {
        public static final int INDEX_MOTOR_ID = 40;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        public final static Measure<Voltage> indexVoltage = Volts.of(5);
        public final static Measure<Voltage> trapIndexVoltage = Volts.of(2);
        public final static Measure<Voltage> indexShootVoltage = Volts.of(-16);
    }

    public static class IntakerConstants {
        public static final int INTAKE_MOTOR_ID = 30;

        public static final Measure<Voltage> intakeVoltage = Volts.of(-7.5);
    }

    public static class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 41;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 42;
        public static final int ARM_MOTOR_ID = 43;
        public static final int PULLER_MOTOR_ID = 44;

        // Shooter gains when deploying shooter to desired angle
        public static final Slot0Configs armGainsUp = new Slot0Configs()
                .withKP(60)
                .withKI(0.01)
                .withKD(0.02)
                .withKV(0.12) // add 12v for desired velocity
                .withKS(0.25); // add 0.24v to overcome friction

        public static final Measure<Current> armZeroCurrent = Amps.of(1.2);
        public static final Measure<Voltage> armZeroVoltage = Volts.of(-2);

        public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(30)
                .withMotionMagicJerk(70)
                .withMotionMagicCruiseVelocity(60);
        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(90d / 24 * 90 / 24 * 84 / 14);
        public static final FeedbackConfigs pullerfeedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(8d / 64 * 16 / 64);

        public static final Measure<Voltage> shooterConstantVoltage = Volts.of(-2);
        public static Measure<Voltage> defaultShootVoltage = Volts.of(-9);
        public static final Measure<Voltage> shortShootVoltage = Volts.of(-8);
        public static final Measure<Distance> shortShootMaxDistance = Meters.of(2.7);
        public static final Measure<Voltage> farShootVoltage = Volts.of(-11);
        public static final Measure<Voltage> pullVoltage = Volts.of(-6);

        public static Measure<Angle> speakerArmOffsetNear = Degrees.of(-43);
        public static Measure<Angle> speakerArmOffset = Degrees.of(-59);
        public static Measure<Angle> speakerArmOffsetFar = Degrees.of(-34);

        public final static Measure<Angle> ampDeployAngle = Degrees.of(190);
        public static final Measure<Voltage> shooterUpDownVoltage = Volts.of(-4);
        public static final Measure<Voltage> shooterIndexVoltage = Volts.of(13);
    }

    public static class BeamBreakConstants {
        public final static int INTAKER_BEAM_BREAK_ID = 1;
        public final static int INDEXER_BEAM_BREAK_ID = 3;
        public final static int SHOOTER_BEAM_BREAK_ID = 2;
    }

    public static class IndicatorConstants {
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 17;
    }

    public static class Logger {
        public static final boolean ENABLE_DEBUG = true;

        public static void debug(String... texts) {
            if (ENABLE_DEBUG) {
                System.out.println(String.join(" ", texts));
            }
        }
    }
}
