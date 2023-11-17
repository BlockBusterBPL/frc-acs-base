package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateTracker;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.lib.drive.AutoAlignPointSelector;
import frc.robot.lib.leds.LEDState;
import frc.robot.lib.leds.TimedLEDState;
import frc.robot.lib.util.TimeDelayedBoolean;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.arm.ArmState.Action;
import frc.robot.subsystems.arm.ArmState.ArmSend;
import frc.robot.subsystems.leds.LED;

public class Arm extends SubsystemBase {

    public enum GameObjectType {
        CUBE, CONE
    }

    public enum GoalState {
        STOW(new ArmState(0, 0, 0, 0, 0, 0, Action.NEUTRAL, ArmSend.LOW)),
        TRANSPORT(new ArmState(0, 0, 0, 0, 0, 0, Action.NEUTRAL, ArmSend.MEDIUM)),
        INTAKE_CUBE_GROUND(new ArmState(0, 0, 0, 0, 0, 0, Action.INTAKING, ArmSend.LOW)),
        INTAKE_CUBE_SHELF(new ArmState(0, 0, 0, 0, 0, 0, Action.INTAKING, ArmSend.MEDIUM)),
        INTAKE_CONE_GROUND(new ArmState(0, 0, 0, 0, 0, 0, Action.INTAKING, ArmSend.MEDIUM)),
        INTAKE_CONE_SHELF(new ArmState(0, 0, 0, 0, 0, 0, Action.INTAKING, ArmSend.MEDIUM)),
        INTAKE_WAIT_SHELF(new ArmState(0, 0, 0, 0, 0, 0, Action.NEUTRAL, ArmSend.FULL)),
        SCORE_CUBE_LOW(new ArmState(0, 0, 0, 0, 0, 0, Action.SCORING, ArmSend.FULL)),
        SCORE_CUBE_MID(new ArmState(0, 0, 0, 0, 0, 0, Action.SCORING, ArmSend.FULL)),
        SCORE_CUBE_HIGH(new ArmState(0, 0, 0, 0, 0, 0, Action.SCORING, ArmSend.MEDIUM)),
        SCORE_CONE_LOW(new ArmState(0, 0, 0, 0, 0, 0, Action.SCORING, ArmSend.FULL)),
        SCORE_CONE_MID(new ArmState(0, 0, 0, 0, 0, 0, Action.SCORING, ArmSend.FULL)),
        SCORE_CONE_HIGH(new ArmState(0, 0, 0, 0, 0, 0, Action.SCORING, ArmSend.MEDIUM)),
        DUMMY_POSITION(new ArmState(0, 0, 0, 0, 0, 0, Action.NEUTRAL, ArmSend.MEDIUM));
        
        public ArmState state;

        GoalState(ArmState goalState) {
            this.state = goalState;
        }
    }

    private ArmMotionPlanner mMotionPlanner;

    private ArmState measuredState = new ArmState();
    private ArmState expectedState = new ArmState();
    private ArmState commandedState = new ArmState();
    private GoalState goalState = GoalState.STOW;
    private GoalState lastGoalState = GoalState.STOW;
    private GameObjectType gameObject = GameObjectType.CUBE;
    private boolean hasBeenHomed = false;
    private TimeDelayedBoolean ensureScoringFinished = new TimeDelayedBoolean();
    private TimeDelayedBoolean ensureIntakeFinished = new TimeDelayedBoolean();
    private boolean resetMotionPlanner = false;

    public static final double kLEDClosenessDeadbandMeters = 0.03;

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
        gripperIO.updateInputs(gripperInputs);
        Logger.getInstance().processInputs("Arm", armInputs);
        Logger.getInstance().processInputs("Gripper", gripperInputs);

        double timestamp = Timer.getFPGATimestamp();

        Optional<TimedLEDState> ledState = handleLEDs(timestamp);
        if (ledState.isPresent()) {
            if (DriverStation.isAutonomousEnabled()) {
                LED.setWantedAction(LED.WantedAction.DISPLAY_VISION);
            } else {
                LED.setDeliveryLEDState(ledState.get());
                LED.setWantedAction(LED.WantedAction.DISPLAY_DELIVERY);
            }
        } else {
            LED.setWantedAction(LED.WantedAction.OFF);
        }

        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            for (Double l : armInputs.tiltSuppliedCurrentAmps) {
                simCurrent += l;
            }
            for (Double l : armInputs.extendSuppliedCurrentAmps) {
                simCurrent += l;
            }
            simCurrent += armInputs.wristSuppliedCurrentAmps;
            for (Double l : gripperInputs.motorCurrentAmps) {
                simCurrent += l;
            }

            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        Rotation2d tiltAngle = Rotation2d.fromRotations(armInputs.tiltRotations);
        Rotation2d wristAngle = Rotation2d.fromRotations(armInputs.wristRotations);

        sensorElevator.setAngle(tiltAngle);
        sensorElevator.setLength(ARM_BASE_LENGTH + armInputs.extendMeters);
        sensorWrist.setAngle(wristAngle.minus(Rotation2d.fromDegrees(90)));

        measuredState = new ArmState(tiltAngle.getRadians(), armInputs.extendMeters, tiltAngle.getRadians(), commandedState.action, commandedState.send);

        // calculate next ArmState from current ArmState planner:update(currentstate)
        ArmState nextArmState = mMotionPlanner.update(measuredState);
        commandedState = nextArmState;
        expectedState = commandedState;

        // set correct game object type
        gripperIO.setGameObject(gameObject);

        // interpret and execute action from next arm state
        switch (commandedState.action) {
            case INTAKING:
                if (!isDoneIntaking()) {
                    gripperIO.setMotor(-1);
                } else {
                    gripperIO.setMotor(0);
                }
                break;
            case SCORING:
                if (atGoal() && !isDoneScoring()) {
                    gripperIO.setMotor(1);
                } else {
                    gripperIO.setMotor(0);
                }
                break;
            case NEUTRAL:
            default:
                gripperIO.setMotor(0);
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

    public boolean getResetMotionPlanner() {
        return resetMotionPlanner;
    }

    public boolean atGoal() {
        return mMotionPlanner.isFinished();
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

    public GameObjectType getGameObject() {
        return gameObject;
    }

    public boolean gripperHasGamepiece() {
        return gripperInputs.coneInIntake || gripperInputs.cubeInIntake;
    }

    public boolean isDoneIntaking() {
        return ensureIntakeFinished.update(gripperHasGamepiece(), 0.75);
    }

    public boolean isDoneScoring() {
        return ensureScoringFinished.update(!gripperHasGamepiece(), 0.75);
    }

    public boolean gameObjectIsCone() {
        return this.gameObject == GameObjectType.CONE;
    }

    public void setGameObject(GameObjectType gameObject) {
        this.gameObject = gameObject;
    }

    public void swapGameObject() {
        if (gameObjectIsCone()) {
            gameObject = GameObjectType.CUBE;
        } else {
            gameObject = GameObjectType.CONE;
        }
    }

    private synchronized Optional<TimedLEDState> handleLEDs(double timestamp) {
        Optional<TimedLEDState> signal = handleScoringAlignmentLEDs(timestamp);
        if (signal.isEmpty()) {
            signal = handleIntakingLEDs(timestamp);
        }
        return signal;
    }

    private synchronized Optional<TimedLEDState> handleIntakingLEDs(double timestamp) {
        Optional<TimedLEDState> display = Optional.empty();
        boolean hasPiece = false;
        switch (gameObject) {
            case CONE:
                hasPiece = gripperHasGamepiece();
                display = Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasCone : TimedLEDState.BlinkingLEDState.kConeIntakeWaiting);
                break;
            case CUBE:
                hasPiece = gripperHasGamepiece();
                display = Optional.of(hasPiece ? TimedLEDState.StaticLEDState.kHasCube : TimedLEDState.BlinkingLEDState.kCubeIntakeWaiting);
                break;
        }
        return display;
    }

    private synchronized Optional<TimedLEDState> handleScoringAlignmentLEDs(double timestamp) {
        AutoAlignPointSelector.RequestedAlignment alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO;
        LEDState pieceColor = LEDState.kAutoAlign;
        double maxError = 0.56 / 2.0; // m (distance between low goals)
        switch (gameObject) {
            case CONE:
                alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO_CONE;
                pieceColor = LEDState.kIntakingCone;
                maxError = 1.13 / 2.0; // m (distance between max cones)
                break;
            case CUBE:
                alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO_CUBE;
                pieceColor = LEDState.kIntakingCube;
                maxError = 1.68 / 2.0; // m (distance between tags)
                break;
        }

        if (goalState == GoalState.SCORE_CONE_LOW || goalState == GoalState.SCORE_CUBE_LOW) {
            // if we aren't low scoring, we need to pick cones or cubes for alignment hinter
            alignmentType = AutoAlignPointSelector.RequestedAlignment.AUTO;
        }
        // Pose2d fieldToVehicle = RobotState.getInstance().getFieldToVehicleAbsolute(timestamp);
        Pose2d fieldToVehicle = RobotStateTracker.getInstance().getCurrentRobotPose();
        Optional<Pose2d> targetPoint = AutoAlignPointSelector.chooseTargetPoint(fieldToVehicle, alignmentType);

        if (targetPoint.isEmpty()) {
            // the target point will be empty if we are too far away from alignment, and we shouldn't hint alignment
            return Optional.empty();
        } else {
            Translation2d error = targetPoint.get().getTranslation().plus(fieldToVehicle.getTranslation().unaryMinus());
            double horizError = Math.abs(error.getY());
            boolean auto_align_active = RobotStateTracker.getInstance().getAutoAlignActive();
            boolean auto_align_on_target = RobotStateTracker.getInstance().getAutoAlignComplete();
            if ((horizError < kLEDClosenessDeadbandMeters && !auto_align_active) || (auto_align_active && auto_align_on_target)) {
                return Optional.of(TimedLEDState.StaticLEDState.kAtAlignment);
            } else {
                if (horizError <= kLEDClosenessDeadbandMeters) {
                    horizError = 0.0;
                }
                double percentage = Util.limit((maxError - horizError) / maxError, 0.0, 1.0);
                return Optional.of(new TimedLEDState.PercentFullLEDState(percentage, pieceColor));
            }
        }
    }
}
