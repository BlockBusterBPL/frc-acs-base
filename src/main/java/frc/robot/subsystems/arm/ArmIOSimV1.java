package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.wpilibj.Timer;

public class ArmIOSimV1 implements ArmIO {
    private final TrapezoidProfile.Constraints tiltConstraints;
    private final TrapezoidProfile.Constraints extendConstraints;
    private final TrapezoidProfile.Constraints wristConstraints;

    private TrapezoidProfile.State lastTiltState;
    private TrapezoidProfile.State lastExtendState;
    private TrapezoidProfile.State lastWristState;

    private Optional<TrapezoidProfile> tiltEstimator;
    private Optional<TrapezoidProfile> extendEstimator;
    private Optional<TrapezoidProfile> wristEstimator;

    private Timer tiltTimer;
    private Timer extendTimer;
    private Timer wristTimer;

    public ArmIOSimV1() {
        tiltConstraints = new Constraints(0.5, 1);
        extendConstraints = new Constraints(0.5, 1);
        wristConstraints = new Constraints(0.5, 1);

        lastTiltState = new State();
        lastExtendState = new State();
        lastWristState = new State();

        tiltEstimator = Optional.empty();
        extendEstimator = Optional.empty();
        wristEstimator = Optional.empty();

        tiltTimer = new Timer();
        extendTimer = new Timer();
        wristTimer = new Timer();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.tiltRotations = lastTiltState.position;
        inputs.tiltVelocityRotPerSec = lastTiltState.velocity;
        
        inputs.extendMeters = lastExtendState.position;
        inputs.extendVelocityMetersPerSec = lastExtendState.velocity;
        
        inputs.wristRotations = lastWristState.position;
        inputs.wristVelocityRotPerSec = lastWristState.velocity;
    }

    @Override
    public void updateOutputs() {
        tiltEstimator.ifPresent((p) -> {lastTiltState = p.calculate(tiltTimer.get());});
        extendEstimator.ifPresent((p) -> {lastExtendState = p.calculate(extendTimer.get());});
        wristEstimator.ifPresent((p) -> {lastWristState = p.calculate(wristTimer.get());});
    }

    @Override
    public void setTiltTarget(double rotations) {
        tiltEstimator = Optional.of(new TrapezoidProfile(tiltConstraints, new State(rotations, 0), lastTiltState));
        tiltTimer.restart();
    }

    @Override
    public void setExtendTarget(double meters) {
        extendEstimator = Optional.of(new TrapezoidProfile(extendConstraints, new State(meters, 0), lastExtendState));
        extendTimer.restart();
    }

    @Override
    public void setWristTarget(double rotations) {
        wristEstimator = Optional.of(new TrapezoidProfile(wristConstraints, new State(rotations, 0), lastWristState));
        wristTimer.restart();
    }
}