package net.ironpulse;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;

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
        public static final double DEADBAND = 0.1;

        public static final Measure<Velocity<Distance>> maxLinearSpeed =
                MetersPerSecond.of(4.5);

        public static final Measure<Distance> trackWidthX = Meters.of(0.8);
        public static final Measure<Distance> trackWidthY = Meters.of(0.8);
        public static final Measure<Distance> driveBaseRadius = Meters.of(
                Math.hypot(
                        trackWidthX.divide(2).magnitude(),
                        trackWidthY.divide(2).magnitude()
                )
        );
        public static final Measure<Velocity<Angle>> maxAngularSpeed =
                RadiansPerSecond.of(maxLinearSpeed.magnitude() / driveBaseRadius.magnitude());

        public static final double ODOMETRY_FREQUENCY = 250;

        public static final Measure<Distance> wheelRadius = Meters.of(0.05);


        // Front Left
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        public static final int FRONT_LEFT_ENCODER_ID = 9;
        public static final double FRONT_LEFT_ENCODER_OFFSET = -0.945;
        // private static final double FRONT_LEFT_ENCODER_OFFSET = 0;

        // Front Right
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
        public static final int FRONT_RIGHT_ENCODER_ID = 21;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = -2.464;
        // private static final double FRONT_RIGHT_ENCODER_OFFSET = 0;

        // Back Left
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 14;
        public static final int BACK_LEFT_ENCODER_ID = 20;
        public static final double BACK_LEFT_ENCODER_OFFSET = -0.966;
        // private static final double BACK_LEFT_ENCODER_OFFSET = 0;

        // Back Right
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_ENCODER_ID = 12;
        public static final double BACK_RIGHT_ENCODER_OFFSET = -2.666;
        public static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;
    }

    public static class IndexerConstants {
        public static final int INDEX_MOTOR_ID = 40;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        public final static Measure<Voltage> indexVoltage = Volts.of(5);
    }

    public static class IntakerConstants {
        public static final int INTAKE_MOTOR_ID = 30;

        public static final Measure<Voltage> intakeVoltage = Volts.of(-7.5);
    }

    public static class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 41;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 42;
        public static final int ARM_MOTOR_ID = 43;

        // Shooter gains when deploying shooter to desired angle
        public static final Slot0Configs armGainsUp = new Slot0Configs()
                .withKP(60)
                .withKI(0.01)
                .withKD(0.02)
                .withKV(0.12)
                .withKS(0.25);

        public static final Measure<Current> armZeroCurrent = Amps.of(1.2);
        public static final Measure<Voltage> armZeroVoltage = Volts.of(-2);

        public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(10)
                .withMotionMagicJerk(50)
                .withMotionMagicCruiseVelocity(10);
        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(90d / 24 * 90 / 24 * 84 / 14);

        public static final Measure<Voltage> shooterConstantVoltage = Volts.of(-2);
        public static final Measure<Voltage> defaultShootVoltage = Volts.of(-6);
        public static final Measure<Voltage> shortShootVoltage = Volts.of(-8);
        public static final Measure<Distance> shortShootMaxDistance = Meters.of(2.7);
        public static final Measure<Voltage> farShootVoltage = Volts.of(-10);

        public static final Measure<Angle> speakerArmOffsetNear = Degrees.of(-47);
        public static final Measure<Angle> speakerArmOffset = Degrees.of(-50);
        public static final Measure<Angle> speakerArmOffsetFar = Degrees.of(-36);

        public final static Measure<Angle> ampDeployAngle = Degrees.of(190);
        public static final Measure<Voltage> manualAimingVoltage = Volts.of(2);
    }

    public static class BeamBreakConstants {
        public final static int INTAKER_BEAM_BREAK_ID = 1;
        public final static int INDEXER_BEAM_BREAK_ID = 3;
        public final static int SHOOTER_BEAM_BREAK_ID = 2;
    }

    public static class IndicatorConstants {
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 15;
    }
}
