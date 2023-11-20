package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.*;
import edu.wpi.first.wpilibj.Timer;

public class ArmIOSimV1 implements ArmIO {
    private final TrapezoidProfile.Constraints kTiltConstraints;
    private final TrapezoidProfile.Constraints kExtendConstraints;
    private final TrapezoidProfile.Constraints kWristConstraints;

    private TrapezoidProfile.State mLastTiltState;
    private TrapezoidProfile.State mLastExtendState;
    private TrapezoidProfile.State mLastWristState;

    private Optional<TrapezoidProfile> mTiltEstimator;
    private Optional<TrapezoidProfile> mExtendEstimator;
    private Optional<TrapezoidProfile> mWristEstimator;

    private Timer mTiltTimer;
    private Timer mExtendTimer;
    private Timer mWristTimer;

    public ArmIOSimV1() {
        kTiltConstraints = new Constraints(0.1875, 0.75);
        kExtendConstraints = new Constraints(1.5, 4);
        kWristConstraints = new Constraints(1.75, 3);

        mLastTiltState = new State();
        mLastExtendState = new State();
        mLastWristState = new State();

        mTiltEstimator = Optional.empty();
        mExtendEstimator = Optional.empty();
        mWristEstimator = Optional.empty();

        mTiltTimer = new Timer();
        mExtendTimer = new Timer();
        mWristTimer = new Timer();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.tiltRotations = mLastTiltState.position;
        inputs.tiltVelocityRotPerSec = mLastTiltState.velocity;
        
        inputs.extendMeters = mLastExtendState.position;
        inputs.extendVelocityMetersPerSec = mLastExtendState.velocity;
        
        inputs.wristRotations = mLastWristState.position;
        inputs.wristVelocityRotPerSec = mLastWristState.velocity;
    }

    @Override
    public void updateOutputs() {
        mTiltEstimator.ifPresent((p) -> {mLastTiltState = p.calculate(mTiltTimer.get());});
        mExtendEstimator.ifPresent((p) -> {mLastExtendState = p.calculate(mExtendTimer.get());});
        mWristEstimator.ifPresent((p) -> {mLastWristState = p.calculate(mWristTimer.get());});
    }

    @Override
    public void setTiltTarget(double rotations) {
        mTiltEstimator = Optional.of(new TrapezoidProfile(kTiltConstraints, new State(rotations, 0), mLastTiltState));
        mTiltTimer.restart();
    }

    @Override
    public void setExtendTarget(double meters) {
        mExtendEstimator = Optional.of(new TrapezoidProfile(kExtendConstraints, new State(meters, 0), mLastExtendState));
        mExtendTimer.restart();
    }

    @Override
    public void setWristTarget(double rotations) {
        mWristEstimator = Optional.of(new TrapezoidProfile(kWristConstraints, new State(rotations, 0), mLastWristState));
        mWristTimer.restart();
    }
}