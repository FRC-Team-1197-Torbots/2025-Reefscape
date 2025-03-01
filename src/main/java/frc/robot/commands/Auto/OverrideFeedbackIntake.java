// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import static frc.robot.util.Helpers.betterModulus;
import static frc.robot.util.Helpers.tan;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Helpers;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class OverrideFeedbackIntake extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem m_robotDrive;
  private final PIDController m_xController;
  private final PIDController m_yController;
  private final String limelightName = VisionConstants.ElevatorLimelightName;
  private final ProfiledPIDController m_turnPID;
  private final CoralMaster m_coralMaster;

  /**
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public OverrideFeedbackIntake(DriveSubsystem subsystem, CoralMaster coralMaster) {
    m_robotDrive = subsystem;
    m_xController = new PIDController(DriveConstants.xTranslationkP, DriveConstants.xTranslationkI, DriveConstants.xTranslationkD);
    m_yController = new PIDController(DriveConstants.yTranslationkP, DriveConstants.yTranslationkI, DriveConstants.yTranslationkD);
    m_turnPID = new ProfiledPIDController(DriveConstants.TurnkP, DriveConstants.TurnkI, DriveConstants.TurnkD, new Constraints(DriveConstants.TurnMaxVelocity, DriveConstants.TurnMaxAccel));
    m_xController.setIZone(0.08); 
    m_yController.setIZone(0.08);
    m_turnPID.enableContinuousInput(0, 360);
    m_coralMaster = coralMaster;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralMaster);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralMaster.setIntake();
    m_yController.setSetpoint(Constants.IntakeAlignDistance);
    m_yController.setTolerance(0.0175);
    m_xController.setSetpoint(m_robotDrive.getStationOffset());
    m_turnPID.setGoal(m_robotDrive.getStationAngle());
    m_robotDrive.setAlignedToReef(false);
    PPHolonomicDriveController.overrideXYFeedback(() -> getFieldRelativeSpeeds().vxMetersPerSecond, () -> getFieldRelativeSpeeds().vyMetersPerSecond);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralMaster.stopIntake();
    PPHolonomicDriveController.clearFeedbackOverrides();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_coralMaster.coralStored();
  }
  
  private ChassisSpeeds getFieldRelativeSpeeds() {
    double yOutput = 0, xOutput = 0;
    if (m_coralMaster.useIntakeAutoAlign() && LimelightHelpers.getTV(limelightName)) {
      double yDistanceFromTag = Helpers.tyToDistance(limelightName);
      double xInput = yDistanceFromTag * tan(LimelightHelpers.getTX(limelightName)); 
      xOutput = -m_xController.calculate(-xInput);
      yOutput = -m_yController.calculate(yDistanceFromTag);

      if (m_yController.atSetpoint()) {
        yOutput = 0;
      }
    }
    return ChassisSpeeds.fromRobotRelativeSpeeds(Math.min(yOutput, 0.225), Math.min(xOutput, 0.3), 0, m_robotDrive.getRotation()) ; // yes y and x are flipped
  }
}
