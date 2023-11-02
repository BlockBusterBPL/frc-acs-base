package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.lib.util.TimeDelayedBoolean;
import frc.robot.subsystems.arm.ArmState.Action;
import frc.robot.subsystems.arm.ArmState.ArmSend;

public class Arm extends SubsystemBase {

    public enum GameObjectType {
        CUBE, CONE
    }

    public enum GoalState {
        STOW(new ArmState(0, 0, 0, 0, 0, 0, Action.NEUTRAL, ArmSend.MEDIUM));
        
        public ArmState state;

        GoalState(ArmState goalState) {
            this.state = goalState;
        }
    }

    private ArmMotionPlanner mMotionPlanner;

    private ArmState measuredState;
    private ArmState expectedState;
    private ArmState commandedState;
    private GoalState goalState;
    private GoalState lastGoalState;
    private static boolean isConeMode = false;
    private boolean hasBeenHomed = false;
    private TimeDelayedBoolean notHasGamepiece = new TimeDelayedBoolean();

    public static final double ARM_BASE_LENGTH = Units.inchesToMeters(19); // distance from arm pivot to vertical
                                                                           // extension of wrist pivot
    public static final double ARM_MASS_OFFSET = 0.0; // FF Amps required to hold elevator horizontal at minimum
                                                      // extension
    public static final double ARM_MASS_DISTANCE_FACTOR = 0.0; // Additional FF Amps to hold horizontal required per
                                                               // meter of extension
    public static final double ARM_CONE_BOOST = 0.0; // FF Amps to add to tilt when wrist is at full extension
    public static final double ELEVATOR_MASS_FACTOR = 0.0; // FF Amps to hold elevator carriage in vertical position
    public static final double WRIST_MASS_FACTOR = 0.0; // FF Amps to hold wrist in horizontal position

    private final Mechanism2d sensorMech = new Mechanism2d(1.75, 1.75);
    private final MechanismRoot2d sensorRoot = sensorMech.getRoot("Main Pivot", 0.25, 0.25);
    private final MechanismLigament2d sensorElevator = sensorRoot
            .append(new MechanismLigament2d("Elevator", ARM_BASE_LENGTH, 0, 5, new Color8Bit(Color.kOrange)));
    private final MechanismLigament2d sensorOffsetPlate = sensorElevator
            .append(new MechanismLigament2d("Offset Plate", 0.08, 270, 5, new Color8Bit(Color.kGray)));
    private final MechanismLigament2d sensorWrist = sensorOffsetPlate
            .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12), 100, 5, new Color8Bit(Color.kPurple)));

    private final Mechanism2d targetMech = new Mechanism2d(1.75, 1.75);
    private final MechanismRoot2d targetRoot = targetMech.getRoot("Main Pivot", 0.25, 0.25);
    private final MechanismLigament2d targetElevator = targetRoot
            .append(new MechanismLigament2d("Elevator", ARM_BASE_LENGTH, 0, 5, new Color8Bit(Color.kOrange)));
    private final MechanismLigament2d targetOffsetPlate = targetElevator
            .append(new MechanismLigament2d("Offset Plate", 0.08, 270, 5, new Color8Bit(Color.kGray)));
    private final MechanismLigament2d targetWrist = targetOffsetPlate
            .append(new MechanismLigament2d("Wrist", Units.inchesToMeters(12), 100, 5, new Color8Bit(Color.kPurple)));

    private ArmIO armIO;
    private ArmIOInputsAutoLogged armInputs;

    private GripperIO gripperIO;
    private GripperIOInputsAutoLogged gripperInputs;

    public Arm(ArmIO armIO, GripperIO gripperIO) {
        mMotionPlanner = new ArmMotionPlanner();

        this.armIO = armIO;
        armInputs = new ArmIOInputsAutoLogged();

        this.gripperIO = gripperIO;
        gripperInputs = new GripperIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        armIO.updateInputs(armInputs);
        Logger.getInstance().processInputs("Arm", armInputs);

        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            for (Double l : armInputs.tiltSuppliedCurrentAmps) {
                simCurrent += l;
            }
            for (Double l : armInputs.extendSuppliedCurrentAmps) {
                simCurrent += l;
            }
            simCurrent += armInputs.wristSuppliedCurrentAmps;

            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        sensorElevator.setAngle(Rotation2d.fromRotations(armInputs.tiltRotations));
        sensorElevator.setLength(ARM_BASE_LENGTH + armInputs.extendMeters);
        sensorWrist.setAngle(Rotation2d.fromRotations(armInputs.wristRotations).minus(Rotation2d.fromDegrees(90)));

        // calculate next ArmState from current ArmState planner:update(currentstate)
        ArmState nextArmState = mMotionPlanner.update(measuredState);
        commandedState = nextArmState;

        // interpret and execute action from next arm state
        switch (commandedState.action) {
            case INTAKING:
                break;
            case NEUTRAL:
                break;
            case SCORING:
                break;
            default:
                break;
        }

        targetElevator.setAngle(Rotation2d.fromRotations(commandedState.tilt));
        targetElevator.setLength(ARM_BASE_LENGTH + commandedState.extend);
        targetWrist.setAngle(Rotation2d.fromRotations(commandedState.wrist).minus(Rotation2d.fromDegrees(90)));

        Logger.getInstance().recordOutput("Arm/MeasuredPositions", sensorMech);
        Logger.getInstance().recordOutput("Arm/TargetPositions", targetMech);

        armIO.setTiltTarget(commandedState.tilt);
        armIO.setTiltFeedForward(calcTiltFeedforward());

        armIO.setExtendTarget(commandedState.extend);
        armIO.setExtendFeedForward(calcExtendFeedForward());

        armIO.setWristTarget(commandedState.wrist);
        armIO.setWristFeedForward(calcWristFeedForward());

        armIO.updateOutputs();
    }

    public GoalState getGoalState() {
        return goalState;
    }

    public GoalState getLastGoalState() {
        return lastGoalState;
    }

    public void setGoalState(GoalState goalState) {
        lastGoalState = this.goalState;
        this.goalState = goalState;

        mMotionPlanner.setDesiredState(this.goalState.state, measuredState);
    }

    private Rotation2d calcWristRefAngle() {
        return Rotation2d.fromRotations(armInputs.wristRotations).minus(Rotation2d.fromRotations(armInputs.tiltRotations));
    }

    private double calcTiltFeedforward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(armInputs.tiltRotations);
        double extendDistance = armInputs.extendMeters;

        // estimates a value proportional to the torque on each tilt gearbox
        double wristMass = calcWristRefAngle().getCos() * ARM_CONE_BOOST;
        double tiltMass = ARM_MASS_OFFSET + (extendDistance * ARM_MASS_DISTANCE_FACTOR);
        double tiltFFCurrent = (wristMass + tiltMass) * tiltAngle.getCos();

        return tiltFFCurrent;
    }

    private double calcExtendFeedForward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(armInputs.tiltRotations);
        
        // estimates a value proportional to the torque on each extend gearbox
        double extendFeedForward = ELEVATOR_MASS_FACTOR * tiltAngle.getSin();

        return extendFeedForward;
    }

    private double calcWristFeedForward() {
        return calcWristRefAngle().getCos() * WRIST_MASS_FACTOR;
    }

    public static boolean isConeMode() {
        return isConeMode;
    }
}
