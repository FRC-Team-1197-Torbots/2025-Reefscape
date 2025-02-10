// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Auto.AutoAlignToTag;
import frc.robot.commands.Auto.AutoIntakeCoral;
import frc.robot.commands.CoralIntake.PositionCoral;
import frc.robot.commands.CoralMaster.IntakeCoral;
import frc.robot.commands.CoralMaster.Score;
import frc.robot.commands.CoralMaster.SetLevel;
import frc.robot.commands.Drive.AlignToTag;
import frc.robot.commands.Drive.AlignToTag.Direction;
import frc.robot.commands.Drive.AlignWheels;
import frc.robot.commands.Drive.PointAtAngle;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.ClawConstants.WristConstants;
import frc.robot.constants.Configs.ArmConfig;
import frc.robot.commands.Drive.PointAtReef;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralMaster;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.util.Level;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // define controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_mechController = new CommandXboxController(1);
  
  // The robot's subsystems and commands are defined here...
  private final Claw m_claw = new Claw();
  private final Arm m_arm = new Arm();
  private final Elevator m_elevator = new Elevator();
  private final CoralMaster m_coralMaster = new CoralMaster(m_arm, m_elevator, m_claw);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_driverController);

  private final SendableChooser<Command> autoChooser;
  private final Trigger coralStored = new Trigger(m_coralMaster::coralStored);
  private final Trigger isLevelFour = new Trigger(m_coralMaster::isLevelFourOrTwo);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
      // Build an auto chooser. This will use Commands.none() as the default option.
      
      // Configure the trigger bindings
      configureBindings();
      registerAutoCommands();

      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    
      // drive with controller
      m_robotDrive.setDefaultCommand(Commands.runOnce(() -> m_robotDrive.driveWithController(true), m_robotDrive));
    }
  
    /**
     * Use this method to define your trigger->command mappings
     */
    private void configureBindings() {
      coralStored.negate().and(isLevelFour).onTrue(new SetLevel(Level.STORE, m_coralMaster, m_driverController));
      
      // ------------------ Aidan ----------------------------
      m_driverController.rightTrigger(0.4).whileTrue(new IntakeCoral(m_coralMaster, m_robotDrive));
      m_driverController.rightTrigger(0.4).onFalse(Commands.sequence(
        Commands.runOnce(() -> m_coralMaster.setStore()),
        Commands.waitUntil(m_arm::onTarget),
        new PositionCoral(m_claw).onlyIf(coralStored)));

      m_driverController.rightBumper().whileTrue(new PointAtReef(m_robotDrive));
      
      // Score 
      m_driverController.leftBumper().whileTrue(new AlignWheels(m_robotDrive, 90).andThen(new AlignToTag(m_robotDrive)));
        
      m_driverController.povUp().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));

      
      m_driverController.a().whileTrue(Commands.startEnd(() -> m_claw.runVoltage(4), () -> m_claw.runVoltage(0)));
      m_driverController.b().onTrue(new SetLevel(Level.STORE, m_coralMaster, m_driverController));
            
      // ------------------- James ----------------------------
      m_mechController.leftBumper().onTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.LEFT)));
      m_mechController.rightBumper().onTrue(Commands.runOnce(() -> m_robotDrive.setScoringSide(Direction.RIGHT)));

      m_mechController.a().whileTrue(new SetLevel(Level.ONE, m_coralMaster, m_driverController));
      m_mechController.a().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController));

      m_mechController.b().and(m_robotDrive::alignedToReef).whileTrue(new SetLevel(Level.THREE, m_coralMaster, m_driverController));
      m_mechController.b().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController));

      m_mechController.x().and(m_robotDrive::alignedToReef).whileTrue(new SetLevel(Level.TWO, m_coralMaster, m_driverController));
      m_mechController.x().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController));
      
      m_mechController.y().and(m_robotDrive::alignedToReef).onTrue(new SetLevel(Level.FOUR, m_coralMaster, m_driverController));
      // m_mechController.y().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController));
      
      m_mechController.povUp().whileTrue(new SetLevel(Level.TOPALGAE, m_coralMaster, m_driverController));
      m_mechController.povUp().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController));

      m_mechController.povDown().whileTrue(new SetLevel(Level.THREE, m_coralMaster, m_driverController));
      m_mechController.povDown().onFalse(new SetLevel(Level.STORE, m_coralMaster, m_driverController));
      
  }
  public void registerAutoCommands() {
    NamedCommands.registerCommand("Auto Intake", Commands.sequence(new AutoIntakeCoral(m_coralMaster), new PositionCoral(m_claw)));
    NamedCommands.registerCommand("Set Store", new SetLevel(Level.STORE, m_coralMaster, m_driverController));
    NamedCommands.registerCommand("Score L1", new SetLevel(Level.ONE, m_coralMaster, m_driverController));
    NamedCommands.registerCommand("Score L2", new SetLevel(Level.TWO, m_coralMaster, m_driverController));
    NamedCommands.registerCommand("Score L3", new SetLevel(Level.THREE, m_coralMaster, m_driverController));
    NamedCommands.registerCommand("Score L4", new SetLevel(Level.TWO, m_coralMaster, m_driverController).until(coralStored.negate()));
    NamedCommands.registerCommand("Align to Reef", new AutoAlignToTag(m_robotDrive));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setVortexArmEncoder() {
    m_arm.resetVortexEncoder();
  }

  public void autoInit() {
      m_arm.resetSetpoint();
      m_elevator.resetSetpoint();
      m_claw.resetSetpoint();
  }

  public void teleopInit() {
    m_arm.resetSetpoint();
    m_arm.setTargetAngle(0);
    m_elevator.resetSetpoint();
    m_claw.resetSetpoint();
  }
}
