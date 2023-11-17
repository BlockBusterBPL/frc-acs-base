package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.GoalState;

public class ArmCommandFactory {
    public static final Command toggleGamepiece(Arm arm) {
        return new InstantCommand(arm::swapGameObject, arm);
    }

    public static final Command waitForGamepieceThenRetract(Arm arm) {
        return new SequentialCommandGroup(
            new WaitUntilCommand(() -> arm.gripperHasGamepiece()),
            new ArmSetGoalTillFinished(arm, GoalState.TRANSPORT)
        );
    }

    public static final Command intakeRetract(Arm arm) {
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

    public static final Command shelfIntakeOpen(Arm arm) {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                new ArmSetGoalTillFinished(arm, GoalState.INTAKE_WAIT_SHELF),
                new WaitUntilCommand(() -> true), // this should wait until we are close enough
                new ConditionalCommand(
                    new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_SHELF), 
                    new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_SHELF), 
                    arm::gameObjectIsCone
                )
            ), 
            new ConditionalCommand(
                new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CONE_SHELF), 
                new ArmSetGoalTillFinished(arm, GoalState.INTAKE_CUBE_SHELF), 
                arm::gameObjectIsCone
            ), 
            () -> false // this should also check if we are close enough
        );
    }
}
