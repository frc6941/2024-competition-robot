package net.ironpulse;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final Mode currentMode = Mode.REAL;

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

    public static final String CAN_BUS_NAME = "6941CANivore1";

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static class SwerveConstants {
        // The max speed of the swerve (should not larger than speedAt12Volts)
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(4.5);
        // The max turning speed of the swerve
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        // The swerve heading (used in SpeakerAimingCommand) gains
        public static final Slot0Configs headingGains = new Slot0Configs()
                .withKP(0.07)
                .withKI(0)
                .withKD(0);

        // The stator current at which the wheels start to slip
        public static final Measure<Current> slipCurrent = Amps.of(300.0);

        // Theoretical free speed (m/s) at 12v applied output;
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(4.5);

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        private static final double COUPLE_RATIO = 3.5;

        public static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        public static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

        // Estimated at first, then fudge-factored to make odom match record
        public static final Measure<Distance> wheelRadius = Meters.of(0.05);
        public static final boolean STEER_MOTOR_REVERSED = true;

        // private static final String CAN_BUS_NAME = "rio";

        public static final int PIGEON_ID = 1;

        // Front Left
        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 5;
        public static final int FRONT_LEFT_STEER_MOTOR_ID = 3;
        public static final int FRONT_LEFT_ENCODER_ID = 9;
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.179688;
        // private static final double FRONT_LEFT_ENCODER_OFFSET = 0;

        // Front Right
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
        public static final int FRONT_RIGHT_ENCODER_ID = 21;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.376709;
        // private static final double FRONT_RIGHT_ENCODER_OFFSET = 0;

        // Back Left
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 14;
        public static final int BACK_LEFT_ENCODER_ID = 20;
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.167969;
        // private static final double BACK_LEFT_ENCODER_OFFSET = 0;

        // Back Right
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
        public static final int BACK_RIGHT_ENCODER_ID = 12;
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0.431641;
        // private static final double BACK_RIGHT_ENCODER_OFFSET = 0;
    }

    public static class IndexerConstants {
        public static final int INDEXER_MOTOR_ID = 40;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final Measure<Voltage> indexVoltage = Volts.of(5);
    }

    public static class ShooterConstants {
        public static final int SHOOTER_L_MOTOR_ID = 41;
        public static final int SHOOTER_R_MOTOR_ID = 42;
        public static final int ARM_MOTOR_ID = 43;

        // Shooter gains when deploying shooter to desired angle
        public static final Slot0Configs armGainsUp = new Slot0Configs()
                .withKP(60)
                .withKI(0.01)
                .withKD(0.02)
                .withKV(0.12)
                .withKS(0.25);

        // Shooter gains when deploying shooter to desired angle
        public static final Slot1Configs armGainsDown = new Slot1Configs()
                .withKP(30)
                .withKI(0)
                .withKD(0);

        public static final Measure<Current> armZeroCurrent = Amps.of(1.5);
        public static final Measure<Voltage> armZeroVoltage = Volts.of(-2);

        public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(10)
                .withMotionMagicJerk(50)
                .withMotionMagicCruiseVelocity(10);
        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withSensorToMechanismRatio(90d / 24 * 90 / 24 * 84 / 14);

        public static final Measure<Voltage> shootVoltage = Volts.of(-10);

        public static final Measure<Angle> ampDeployAngle = Degrees.of(190);

        public static final Measure<Angle> speakerArmOffset = Degrees.of(-20);

        public static final Measure<Voltage> manualAimingVoltage = Volts.of(2);
    }

    public static class IntakerConstants {
        public static final int INTAKER_MOTOR_ID = 30;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        public static final Measure<Voltage> intakeVoltage = Volts.of(-8);
    }

    public static class BeamBreakConstants {
        public static final int INTAKER_BEAM_BREAK_ID = 1;
        public static final int INDEXER_BEAM_BREAK_ID = 3;
        public static final int SHOOTER_BEAM_BREAK_ID = 2;
    }

    public static class IndicatorConstants {
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 15;
    }
}
