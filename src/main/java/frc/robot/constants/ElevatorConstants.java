package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    public static final int leftMotorId = 10;
    public static final int rightMotorId = 11;
    
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kG = 0;

    public static final double MinHeight = 25;
    public static final double MaxHeight = 0;
    public static final double Tolerance = 1;
    public static final double ApproachingTargetThreshold = 0.9;

    // from block cad
    public static final double Store = Units.inchesToMeters(0);
    public static final double Station = Units.inchesToMeters(1);
    public static final double L1 = Units.inchesToMeters(3.559);
    public static final double L2 = Units.inchesToMeters(0.4);
    public static final double L3 = Units.inchesToMeters(9);
    public static final double L4 = Units.inchesToMeters(24);
    public static final double DeAlgaeL3 = Units.inchesToMeters(12);
    public static final double Net = Units.inchesToMeters(25);
    

    public static final double RadiusMeters = Units.inchesToMeters(1.1 / 2); // ask alex what this should be
    public static final double MotorReduction = 58.0 / 7.0; 
}