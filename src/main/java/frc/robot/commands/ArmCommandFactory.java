package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;
import frc.robot.subsystems.drive.Drive;

public class ArmCommandFactory {
    public static final Command toggleGamepiece(Arm arm) {
        return new InstantCommand(arm::swapGameObject, arm);
    }

    public static final Command waitForIntakeThenRetract(Arm arm) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(arm::isDoneIntaking),
            new ArmSetGoalTillFinished(arm, GoalState.TRANSPORT)
        );
    }

    public static final Command waitForScoreThenRetract(Arm arm) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(arm::isDoneScoring),
            new ArmSetGoalTillFinished(arm, GoalState.STOW)
        );
    }

    public static final Command retract(Arm arm) {
        return new ConditionalCommand(
            new ArmSetGoalTillFinished(arm, GoalState.TRANSPORT), 
            new ArmSetGoalTillFinished(arm, GoalState.STOW), 
            arm::gripperHasGamepiece
        );
    }

    public static final Command groundIntakeOpen(Arm arm) { 
        return new ConditionalCommand(
            new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_GROUND), 
            new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_GROUND), 
            arm::gameObjectIsCone
        );
    }

    public static final Command shelfIntakeOpen(Arm arm, Drive drive) {
        return 
        // new ConditionalCommand(
            // new ConditionalCommand(
            //     new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_SHELF), 
            //     new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_SHELF), 
            //     arm::gameObjectIsCone
            // ), 
            new SequentialCommandGroup(
                // new ArmSetGoalTillFinished(arm, GoalState.INTAKE_WAIT_SHELF),
                // new WaitUntilCommand(drive::autoAlignAtTarget), // this should wait until we are close enough
                new ConditionalCommand(
                    new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_SHELF), 
                    new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_SHELF), 
                    arm::gameObjectIsCone
                )
            );//, 
            // drive::autoAlignAtTarget // this should also check if we are close enough
        // );
    }

    public static final Command autoIntakeShelf(Arm arm, Drive drive) {
        return new SequentialCommandGroup(
            shelfIntakeOpen(arm, drive),
            waitForIntakeThenRetract(arm)
        );
    }

    public static final Command autoScore(GoalState goalState, Arm arm, Drive drive) {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new ArmSetGoalTillFinished(arm, goalState),
                waitForScoreThenRetract(arm)
            ), 
            new SequentialCommandGroup(
                new ArmSetGoalTillFinished(arm, GoalState.SCORE_WAIT),
                new WaitUntilCommand(drive::autoAlignAtTarget),
                new ArmSetGoalTillFinished(arm, goalState),
                waitForScoreThenRetract(arm)
            ),
            drive::autoAlignAtTarget
        );
    }

    public static final Command autoScore(GoalState stateIfCube, GoalState stateIfCone, Arm arm, Drive drive) {
        return new ConditionalCommand(
            autoScore(stateIfCone, arm, drive), 
            autoScore(stateIfCube, arm, drive), 
            arm::gameObjectIsCone
        );
    }
}
