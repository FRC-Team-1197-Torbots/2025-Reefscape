package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.util.LimelightHelpers;

public class Vision {

    private SwerveDrivePoseEstimator m_poseEstimator;

    protected Vision(SwerveDrivePoseEstimator poseEstimator) {
        m_poseEstimator = poseEstimator;
    }

    public void updatePoseEstimation(Pigeon2 gyroAngle) {
        addToPoseEstimator(VisionConstants.ReefLightLightName, gyroAngle);
        addToPoseEstimator(VisionConstants.ElevatorLimelightName, gyroAngle);
    }

    private void addToPoseEstimator(String limelightName, Pigeon2 gyro) {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation(limelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if(mt2 == null)
            return;
        if (Math.abs(gyro.getRate()) > 720)  { // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 1 && mt2.avgTagDist > 3) {
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            // stolen from 1678
            double xyStdDev = 4.3 * ((0.01 + (0.08 * Math.pow(mt2.avgTagDist, 2.0))) / mt2.tagCount);

            xyStdDev = Math.max(0.03, xyStdDev);
                        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
                        m_poseEstimator.addVisionMeasurement(
                                mt2.pose,
                                mt2.timestampSeconds);
                    }
                }
}
