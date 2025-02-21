package frc.robot.commands.CoralMaster;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.CoralMaster;
import frc.robot.util.Level;

public class SetLevel extends SequentialCommandGroup {
    
    

    public SetLevel(Level level, CoralMaster coralMaster, CommandXboxController controller, Trigger alignedToReef) {
            addCommands(
                Commands.runOnce(() -> coralMaster.setCurrentLevel(level)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(ArmConstants.Store), coralMaster.getArm()),
                Commands.waitUntil(coralMaster.getArm()::onTarget),
                Commands.runOnce(() -> coralMaster.setState(level.elevatorHeight, level.wristAngle), coralMaster).onlyIf(RobotModeTriggers.teleop()),
                Commands.waitUntil(() -> coralMaster.getElevator().approachingHeight(level)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(level.armAngle), coralMaster.getArm()),
                Commands.waitUntil(alignedToReef.and(coralMaster::onTarget)),
                new Score(coralMaster).onlyIf(() -> level.isReefScoringPosition)
            );
    }
}
