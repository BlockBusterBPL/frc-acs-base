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
        double tilt = 0.0;
        double extend = 0.0;
        double wrist = 0.0;
    }

    public static final double ARM_BASE_LENGTH = Units.inchesToMeters(19); // distance from arm pivot to vertical extension of wrist pivot
    public static final double ARM_MASS_OFFSET = 0.0; // FF Amps required to hold elevator horizontal at minimum extension
    public static final double ARM_MASS_DISTANCE_FACTOR = 0.0; // Additional FF Amps to hold horizontal required per meter of extension
    public static final double ARM_CONE_BOOST = 0.0; // FF Amps to add to tilt when wrist is at full extension
    public static final double ELEVATOR_MASS_FACTOR = 0.0; // FF Amps to hold elevator carriage in vertical position
    public static final double WRIST_MASS_FACTOR = 0.0; // FF Amps to hold wrist in horizontal position

    private final Mechanism2d mech = new Mechanism2d(3, 3);
    private final MechanismRoot2d root = mech.getRoot("Main Pivot", 1, 1);
    private final MechanismLigament2d elevator = root
            .append(new MechanismLigament2d("Elevator", 1, 0, 0.05, new Color8Bit(Color.kOrange)));
    private final MechanismLigament2d offsetPlate = root
            .append(new MechanismLigament2d("Offset Plate", 0.05, -90, 0.05, new Color8Bit(Color.kGray)));
    private final MechanismLigament2d wrist = root
            .append(new MechanismLigament2d("Wrist", 0.375, -90, 0.05, new Color8Bit(Color.kPurple)));

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
            for (Double l : inputs.wristSuppliedCurrentAmps) {
                simCurrent += l;
            }
            Robot.updateSimCurrentDraw(this.getClass().getName(), simCurrent);
        }

        io.updateOutputs();
    }

    private double calcTiltFeedforward() {
        Rotation2d tiltAngle = Rotation2d.fromRotations(inputs.tiltRotations);
        double extendDistance = inputs.extendMeters;
        Rotation2d wristAngleWorldRelative = Rotation2d.fromRotations(inputs.wristRotations).minus(tiltAngle);

        double wristMass = wristAngleWorldRelative.getCos() * ARM_CONE_BOOST;
        double tiltMass = ARM_MASS_OFFSET + (extendDistance * ARM_MASS_DISTANCE_FACTOR);
        double tiltFFCurrent = (wristMass + tiltMass) * tiltAngle.getCos();

        return tiltFFCurrent;
    }
}
