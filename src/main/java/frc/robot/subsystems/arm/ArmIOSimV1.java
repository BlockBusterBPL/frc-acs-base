package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.wpilibj.Timer;

public class ArmIOSimV1 implements ArmIO {
    private final TrapezoidProfile.Constraints tiltConstraints = new Constraints(0.5, 1);
    private final TrapezoidProfile.Constraints extendConstraints = new Constraints(0.5, 1);
    private final TrapezoidProfile.Constraints wristConstraints = new Constraints(0.5, 1);

    private TrapezoidProfile.State lastTiltState = new State();
    private TrapezoidProfile.State lastExtendState = new State();
    private TrapezoidProfile.State lastWristState = new State();

    private Optional<TrapezoidProfile> tiltEstimator = Optional.empty();
    private Optional<TrapezoidProfile> extendEstimator = Optional.empty();
    private Optional<TrapezoidProfile> wristEstimator = Optional.empty();

    private Timer tiltTimer = new Timer();
    private Timer extendTimer = new Timer();
    private Timer wristTimer = new Timer();

    public ArmIOSimV1() {
        
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