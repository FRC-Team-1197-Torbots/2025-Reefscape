package frc.robot.constants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.ClawConstants.CoralIntakeConstants;
import frc.robot.constants.ClawConstants.WristConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
                    turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
                    turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
                }
        }

        public static final class DrivingTalonConfig {
            public static TalonFXConfiguration talonConfig = new TalonFXConfiguration();
            
            static {
                    talonConfig.Slot0.kP = 0.0004;
                    talonConfig.Slot0.kI = 0;
                    talonConfig.Slot0.kD = 0;
                    talonConfig.Slot0.kV = 1 / ModuleConstants.kKrakenDriveFreeSpeedRps;
    
                    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
                    talonConfig.CurrentLimits.SupplyCurrentLimit = 50;
                    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                    double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;

                    talonConfig.Feedback.SensorToMechanismRatio = 1 / drivingFactor; // meters
            }
        } 

        public static final class ElevatorConfig {
                public static final SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
                public static final SparkFlexConfig rightMotorConfig = new SparkFlexConfig();
                
                static {
                        leftMotorConfig.idleMode(IdleMode.kCoast);
                        leftMotorConfig.secondaryCurrentLimit(60);
                        leftMotorConfig.smartCurrentLimit(50);
                        double positionFactor = ElevatorConstants.DiameterMeters * Math.PI * 2 / ElevatorConstants.MotorReduction; // times 2 because cascade
                        leftMotorConfig.encoder.positionConversionFactor(positionFactor); // meters
                        leftMotorConfig.encoder.velocityConversionFactor(positionFactor / 60); // meters per sec
                        leftMotorConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(ElevatorConstants.kP, ElevatorConstants.kI,ElevatorConstants.kD);
                        leftMotorConfig.closedLoop.maxMotion
                                .maxVelocity(ElevatorConstants.MaxVelocity)
                                .maxAcceleration(ElevatorConstants.MaxAcceleration)
                                .allowedClosedLoopError(0.01);
                        rightMotorConfig.apply(leftMotorConfig);
                        rightMotorConfig.follow(ElevatorConstants.leftMotorId,false);
        }
    }

    public static final class ClawConfig {
        public static final SparkFlexConfig wristMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

        static {
                wristMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(WristConstants.kP, WristConstants.kI, WristConstants.kD);
                wristMotorConfig.idleMode(IdleMode.kBrake);
                wristMotorConfig.encoder.positionConversionFactor(WristConstants.MotorGearReduction * 360); // degrees

                intakeMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pidf(CoralIntakeConstants.kP, CoralIntakeConstants.kI, CoralIntakeConstants.kD, CoralIntakeConstants.IntakeVelocityFF);
                intakeMotorConfig.idleMode(IdleMode.kBrake);
        }
    }

    public static final class ArmConfig {
        public static final SparkFlexConfig leftMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig rightMotorConfig = new SparkFlexConfig();

        static {
                leftMotorConfig.idleMode(IdleMode.kBrake);
                leftMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
                leftMotorConfig.externalEncoder.positionConversionFactor(ArmConstants.EncoderReduction * 360);
                rightMotorConfig.apply(leftMotorConfig);
                rightMotorConfig.follow(ArmConstants.LeftMotorId);
        }
    }

}