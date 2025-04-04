package frc.robot.util;

import frc.robot.subsystems.Drive.VisionConstants;

public class Helpers {

    public static boolean isBlue = false;
    public static boolean isOneCoralAway = false;
    public static boolean isAuto = false;
    

    // public static Pose3d get(int id) {
    //     return Constants.aprilTags.getTagPose(id).get();
    // }
    

    public static double betterModulus(double x, double y) {
        return (x % y + y) % y;
    }

    /**
    *   @param degrees input angle in degrees
    *   @return tan of the angle
    */
    public static double tan(double degrees) {
        return Math.tan(Math.toRadians(degrees));
    }
    
    /**
    *   @param degrees input angle in degrees
    *   @return sin of the angle
    */
    public static double sin(double degrees) {
        return Math.sin(Math.toRadians(degrees));
    }

    /**
     * 
     * @return distance from limelight to tag
     */
    public static double tyToDistance(String limelightName) {
        switch (limelightName) {
            case VisionConstants.ReefLightLightName:
                return VisionConstants.TagToLimelightHeightOffset / tan(LimelightHelpers.getTY(limelightName) + VisionConstants.LimelightMountAngle);
            case VisionConstants.ElevatorLimelightName:
                return 0.4649 / tan(LimelightHelpers.getTY(limelightName) + 28);
            default:
                return 0;
        }
    }

    public static double txToDistanceOffset(String limelightName) {
        return tyToDistance(limelightName) * tan(LimelightHelpers.getTX(limelightName));
    }

    
    
}
