package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.9 ;
    public static final double kMaxAngularSpeed = 2.15 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.492); // check this
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.492);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 7;
    public static final int kRearRightTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 9;

    public static final boolean kGyroReversed = false;

    public static final double kDriveDeadband = 0.05;

    public static final double TurnkP = 0.01;
    public static final double TurnkI = 0;
    public static final double TurnkD = 0;
    public static final double TurnMaxVelocity = 360;
    public static final double TurnMaxAccel = 99999;
    
    
    public static double xTranslationkP = 1.2;
    public static double xTranslationkI = 1; // 0.7
    public static double xTranslationkD = 0.06;

    public static double xIntakeTranslationkP = 0.8;//1.2
    public static double xIntakeTranslationkI = 0;
    public static double xIntakeTranslationkD = 0.15;
    
    public static final double xTranslationMaxVel = 2;
    public static final double xTranslationMaxAccel = 3;
    
    public static double yTranslationkP = 0.7;
    public static double yTranslationkI = 0.5;
    public static double yTranslationkD = 0.019;

    public static double stationYTranslationkD = 0.06;
    
    public static final double yTranslationMaxVel = 2;
    public static final double yTranslationMaxAccel = 3;

    public static final double DriveToNetP = 1;
    public static final double DriveToNetI = 0;
    public static final double DriveToNetD = 0;
    public static final double CoralGroundIntakeDistance = 1;

  }

  

  

