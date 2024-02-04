package net.ironpulse;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.*;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static final Mode currentMode = Mode.SIM;

    public static String CAN_BUS_NAME = "6941Canivore1";

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
    }

    public static class IndexerConstants {
        public static final int INDEX_MOTOR_ID = 12;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        public final static Measure<Voltage> indexVoltage = Volts.of(5);
    }

    public static class IntakerConstants {
        public static final int INTAKE_MOTOR_ID = 13;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final Measure<Voltage> intakeVoltage = Volts.of(-8);
    }

    public static class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 14;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 15;
        public static final int ARM_MOTOR_ID = 16;

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

        public static final Measure<Voltage> shooterConstantVoltage = Volts.of(-5);
        public static final Measure<Voltage> shootVoltage = Volts.of(-10);
        public final static Measure<Angle> speakerArmOffset = Degrees.of(-20);
    }

    public static class BeamBreakConstants {
        public final static int INTAKER_BEAM_BREAK_ID = 1;
        public final static int INDEXER_BEAM_BREAK_ID = 2;
        public final static int SHOOTER_BEAM_BREAK_ID = 3;
    }
}
