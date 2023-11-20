// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.MotionMagicVoltage;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.lib.phoenixpro.CANcoderLiveConfigHelper;
import frc.robot.lib.phoenixpro.FalconFeedbackControlHelper;
import frc.robot.lib.phoenixpro.TalonFXLiveConfigHelper;

/** Add your docs here. */
public class FalconSwerveIO implements SwerveModuleIO {
    private final TalonFX mDriveMotor;
    private final TalonFX mSteerMotor;

    private final VelocityVoltage mDriveControl;
    private final DutyCycleOut mDriveControlOpenLoop;
    private boolean mUseOpenLoopDrive = false;

    private final MotionMagicVoltage mSteerControl;
    private final VoltageOut mSteerControlOpenLoop;
    private boolean mUseOpenLoopSteering = false;

    private final TalonFXConfiguration mDriveConfig = new TalonFXConfiguration();
    private final FalconFeedbackControlHelper mDriveFeedbackHelper;

    private final TalonFXConfiguration mSteerConfig = new TalonFXConfiguration();
    private final FalconFeedbackControlHelper mSteerFeedbackHelper;

    private final CANcoder mEncoder;

    private final CANcoderConfiguration mEncoderConfig = new CANcoderConfiguration();

    private StatusSignalValue<Double> mDrivePosition;
    private StatusSignalValue<Double> mDriveVelocity;
    private StatusSignalValue<Double> mDriveSuppliedCurrent;
    private StatusSignalValue<Double> mDriveTempCelsius;
    private StatusSignalValue<Double> mSteerPosition;
    private StatusSignalValue<Double> mSteerVelocity;
    private StatusSignalValue<Double> mSteerSuppliedCurrent;
    private StatusSignalValue<Double> mSteerTempCelsius;

    public FalconSwerveIO(int moduleID, String canbus) {
        mDriveMotor = new TalonFX(10 + moduleID, canbus);
        mSteerMotor = new TalonFX(20 + moduleID, canbus);
        mEncoder = new CANcoder(20 + moduleID, canbus);

        mDriveControl = new VelocityVoltage(0, true, 0, 0, false);
        mDriveControlOpenLoop = new DutyCycleOut(0, false, false);

        mDriveConfig.Slot0.kP = 0.0;
        mDriveConfig.Slot0.kV = 0.0;
        mDriveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        mDriveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        mDriveMotor.getConfigurator().apply(mDriveConfig);
        mDriveFeedbackHelper = new FalconFeedbackControlHelper(mDriveMotor, mDriveConfig.Slot0, null);

        mSteerControl = new MotionMagicVoltage(0, true, 0, 0, false);
        mSteerControlOpenLoop = new VoltageOut(0, true, false);

        mSteerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
        mSteerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -20;
        mSteerConfig.Feedback.FeedbackRemoteSensorID = mEncoder.getDeviceID();
        mSteerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        mSteerConfig.Feedback.RotorToSensorRatio = 1/Constants.kSteerReduction;
        mSteerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        mSteerMotor.getConfigurator().apply(mSteerConfig);
        mSteerFeedbackHelper = new FalconFeedbackControlHelper(mSteerMotor, mSteerConfig.Slot0, mSteerConfig.MotionMagic);

        mEncoder.getConfigurator().refresh(mEncoderConfig);
        mEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        mEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        mEncoder.getConfigurator().apply(mEncoderConfig);

        mDrivePosition = mDriveMotor.getPosition();
        mDriveVelocity = mDriveMotor.getVelocity();
        mDriveSuppliedCurrent = mDriveMotor.getSupplyCurrent();
        mDriveTempCelsius = mDriveMotor.getDeviceTemp();
        mSteerPosition = mSteerMotor.getPosition();
        mSteerVelocity = mSteerMotor.getVelocity();
        mSteerSuppliedCurrent = mSteerMotor.getSupplyCurrent();
        mSteerTempCelsius = mSteerMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        mDrivePosition.refresh();
        mDriveVelocity.refresh();
        mDriveSuppliedCurrent.refresh();
        mDriveTempCelsius.refresh();
        mSteerPosition.refresh();
        mSteerVelocity.refresh();
        mSteerSuppliedCurrent.refresh();
        mSteerTempCelsius.refresh();
        
        double position_compensated = mDrivePosition.getValue() + (mDriveVelocity.getValue() * mDrivePosition.getTimestamp().getLatency());
        double angle_compensated = mSteerPosition.getValue() + (mSteerVelocity.getValue() * mSteerPosition.getTimestamp().getLatency());

        inputs.driveMeters = convertRotationsToMeters(position_compensated);
        inputs.driveVelocityMetersPerSec = convertRotationsToMeters(mDriveVelocity.getValue());
        inputs.driveSuppliedCurrentAmps = mDriveSuppliedCurrent.getValue();
        inputs.driveTempCelsius = mDriveTempCelsius.getValue();

        inputs.steerPositionRotations = angle_compensated;
        inputs.steerVelocityRotPerSec = mSteerVelocity.getValue();
        inputs.steerSuppliedCurrentAmps = mSteerSuppliedCurrent.getValue();
        inputs.steerTempCelsius = mSteerTempCelsius.getValue();
    }

    @Override
    public void updateOutputs() {
        boolean targetSpeedAboveThreshold = mDriveControl.Velocity >= Constants.kMinVelocityForFieldWeakening;
        boolean currentSpeedAboveThreshold = convertRotationsToMeters(mDriveVelocity.getValue()) >= Constants.kMinVelocityForFieldWeakening;
        mDriveControl.EnableFOC = !(targetSpeedAboveThreshold && currentSpeedAboveThreshold && Constants.kUseFieldWeakening);
        
        if (mUseOpenLoopDrive) {
            mDriveMotor.setControl(mDriveControlOpenLoop);
        } else {
            mDriveMotor.setControl(mDriveControl);
        }
        if (mUseOpenLoopSteering) {
            mSteerMotor.setControl(mSteerControlOpenLoop);
        } else {
            mSteerMotor.setControl(mSteerControl);
        }
    }

    private double convertRotationsToMeters(double rotations) {
        double wheelCircumference = Constants.kDriveWheelDiameter * Math.PI;
        double metersPerMotorRotation = wheelCircumference / Constants.kDriveReduction;

        return rotations * metersPerMotorRotation;
    }

    private double convertMetersToRotations(double meters) {
        double wheelCircumference = Constants.kDriveWheelDiameter * Math.PI;
        double motorRotationsPerMeter = Constants.kDriveReduction / wheelCircumference;

        return meters * motorRotationsPerMeter;
    }

    @Override
    public void setDriveSpeedClosedLoop(double speedMetersPerSecond) {
        mDriveControl.Velocity = convertMetersToRotations(speedMetersPerSecond);
    }

    @Override
    public void setDriveThrottleOpenLoop(double throttle) {
        mDriveControlOpenLoop.Output = throttle;
    }

    @Override
    public void setDriveUseOpenLoop(boolean useOpenLoopDrive) {
        mUseOpenLoopDrive = useOpenLoopDrive;
    }

    @Override
    public void setSteerPositionTarget(double steerAngleRotations) {
        mSteerControl.Position = steerAngleRotations;
    }

    @Override
    public void setDriveBrakeMode(boolean driveBrakeMode) {
        TalonFXLiveConfigHelper.editConfig(mDriveMotor, (c) -> {
            c.MotorOutput.NeutralMode = (driveBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            return c;
        });
    }

    @Override
    public void setSteerBrakeMode(boolean steerBrakeMode) {
        TalonFXLiveConfigHelper.editConfig(mSteerMotor, (c) -> {
            c.MotorOutput.NeutralMode = (steerBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            return c;
        });
    }

    @Override
    public void setDriveKP(double driveKP) {
        mDriveFeedbackHelper.setKP(driveKP);
    }

    @Override
    public void setDriveKI(double driveKI) {
        mDriveFeedbackHelper.setKI(driveKI);
    }

    @Override
    public void setDriveKD(double drivekD) {
        mDriveFeedbackHelper.setKD(drivekD);
    }

    @Override
    public void setDriveKV(double drivekF) {
        mDriveFeedbackHelper.setKV(drivekF);
    }

    @Override
    public void setDriveKS(double driveKS) {
        mDriveFeedbackHelper.setKS(driveKS);
    }

    @Override
    public void setSteerKP(double steerKP) {
        mSteerFeedbackHelper.setKP(steerKP);
    }

    @Override
    public void setSteerKI(double steerKI) {
        mSteerFeedbackHelper.setKI(steerKI);
    }

    @Override
    public void setSteerKD(double steerKD) {
        mSteerFeedbackHelper.setKD(steerKD);
    }

    @Override
    public void setSteerKS(double steerKS) {
        mSteerFeedbackHelper.setKS(steerKS);
    }
        
    @Override
    public void setSteerKV(double steerKF) {
        mSteerFeedbackHelper.setKV(steerKF);
    }

    @Override
    public void setSteerUseOpenLoop(boolean enableManualVoltage) {
        mUseOpenLoopSteering = enableManualVoltage;
    }

    @Override
    public void setSteerVoltageOpenLoop(double steerVoltage) {
        mSteerControlOpenLoop.Output = steerVoltage;
    }

    @Override
    public void updateEncoderOffset(double zeroRotations) {
        CANcoderLiveConfigHelper.editConfig(mEncoder, (c) -> {
            c.MagnetSensor.MagnetOffset = -zeroRotations;
            return c;
        });
    }

    @Override
    public double getEncoderOffset() {
        return CANcoderLiveConfigHelper.getValueFromConfig(mEncoder, (c) -> {
            return c.MagnetSensor.MagnetOffset;
        });
    }

    @Override
    public double getEncoderRawPosition() {
        mEncoder.getConfigurator().refresh(mEncoderConfig);
        var currentOffset = mEncoderConfig.MagnetSensor.MagnetOffset;
        return mEncoder.getAbsolutePosition().getValue() - currentOffset;
    }

    @Override
    public void stop() {
        mDriveMotor.stopMotor();
        mSteerMotor.stopMotor();
    }
}
