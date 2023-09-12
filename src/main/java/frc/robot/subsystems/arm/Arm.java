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

public class Arm extends SubsystemBase {

    public static class ArmPosition {
        public final double tilt;
        public final double extend;
        public final double wrist;

        public ArmPosition(double tilt, double extend, double wrist) {
            this.tilt = tilt;
            this.extend = extend;
            this.wrist = wrist;
        }
    }

    public static class PositionPresets {
        public static final ArmPosition EMPTY_STOWED = new ArmPosition(0, 0, 0);

        public static final ArmPosition CUBE_STOWED = new ArmPosition(0, 0, 0);
        // CUBE_PICKUP_GROUND
        // CUBE_PICKUP_RAMP
        // CUBE_PICKUP_SHELF
        // CUBE_SCORE_LOW
        // CUBE_SCORE_MID
        // CUBE_SCORE_HIGH
        // CUBE_SCORE_BACK

        public static final ArmPosition CONE_STOWED = new ArmPosition(0, 0, 0);
        // CONE_PICKUP_GROUND
        // CONE_PICKUP_RAMP
        // CONE_PICKUP_SHELF
        // CONE_SCORE_LOW
        // CONE_SCORE_MID
        // CONE_SCORE_HIGH
    }

    private ArmPosition activePreset = PositionPresets.EMPTY_STOWED;
    private static boolean isConeMode = false;

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

    private ArmIO io;
    private ArmIOInputsAutoLogged inputs;

    public Arm(ArmIO io) {
        this.io = io;
        inputs = new ArmIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Arm", inputs);

        if (Constants.getMode() == Mode.SIM) {
            double simCurrent = 0.0;
            for (Double l : inputs.tiltSuppliedCurrentAmps) {
                simCurrent += l;
            }
            for (Double l : inputs.extendSuppliedCurrentAmps) {
                simCurrent += l;
            }
            simCurrent += inputs.wristSuppliedCurrentAmps;

            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        sensorElevator.setAngle(Rotation2d.fromRotations(inputs.tiltRotations));
        sensorElevator.setLength(ARM_BASE_LENGTH + inputs.extendMeters);
        sensorWrist.setAngle(Rotation2d.fromRotations(inputs.wristRotations).minus(Rotation2d.fromDegrees(90)));

        targetElevator.setAngle(Rotation2d.fromRotations(activePreset.tilt));
        targetElevator.setLength(ARM_BASE_LENGTH + activePreset.extend);
        targetWrist.setAngle(Rotation2d.fromRotations(activePreset.wrist).minus(Rotation2d.fromDegrees(90)));

        Logger.getInstance().recordOutput("Arm/MeasuredPositions", sensorMech);
        Logger.getInstance().recordOutput("Arm/TargetPositions", targetMech);

        io.setTiltTarget(activePreset.tilt);
        io.setTiltFeedForward(calcTiltFeedforward());

        io.setExtendTarget(activePreset.extend);
        io.setExtendFeedForward(calcExtendFeedForward());

        io.setWristTarget(activePreset.wrist);
        io.setWristFeedForward(calcWristFeedForward());

        io.updateOutputs();
    }

    private Rotation2d calcWristRefAngle() {
        return Rotation2d.fromRotations(inputs.wristRotations).minus(Rotation2d.fromRotations(inputs.tiltRotations));
    }

    private double calcTiltFeedforward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(inputs.tiltRotations);
        double extendDistance = inputs.extendMeters;

        // estimates a value proportional to the torque on each tilt gearbox
        double wristMass = calcWristRefAngle().getCos() * ARM_CONE_BOOST;
        double tiltMass = ARM_MASS_OFFSET + (extendDistance * ARM_MASS_DISTANCE_FACTOR);
        double tiltFFCurrent = (wristMass + tiltMass) * tiltAngle.getCos();

        return tiltFFCurrent;
    }

    private double calcExtendFeedForward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(inputs.tiltRotations);
        
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
