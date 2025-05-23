package frc.robot.subsystems.CoralMaster;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.util.Level;

public class SetLevel extends SequentialCommandGroup {
    
    

    public SetLevel(Level level, CoralMaster coralMaster, Trigger alignedToReef) {
            addCommands(
                Commands.runOnce(() -> coralMaster.setCurrentLevel(level)),
                Commands.runOnce(() -> coralMaster.getArm().setTargetAngle(ArmConstants.Store), coralMaster.getArm()),
                Commands.waitUntil(coralMaster.getArm()::onTarget),
                Commands.runOnce(() -> coralMaster.setState(level.withOffset().elevatorHeight, level.withOffset().wristAngle), coralMaster).onlyIf(RobotModeTriggers.teleop()),
                Commands.waitUntil(() -> coralMaster.getElevator().approachingHeight(level)),
                Commands.run(() -> coralMaster.setState(level.withOffset()), coralMaster).until(alignedToReef.and(coralMaster::onTarget)),
                new Score(coralMaster).onlyIf(() -> level.isReefScoringPosition)
            );
    }
}
