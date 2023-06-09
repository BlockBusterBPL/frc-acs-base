// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.lib.CANcoderLiveConfigHelper;
import frc.robot.lib.FalconFeedbackControlHelper;
import frc.robot.lib.TalonFXLiveConfigHelper;

/** Add your docs here. */
public class FalconSwerveIO implements SwerveModuleIO {
    private final TalonFX m_drive;
    private final TalonFX m_steer;

    private final VelocityTorqueCurrentFOC m_driveControl;

    private final PositionTorqueCurrentFOC m_steerControl;

    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final FalconFeedbackControlHelper driveHelper;

    private final TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    private final FalconFeedbackControlHelper steerHelper;

    private final CANcoder m_encoder;

    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    private StatusSignalValue<Double> m_drivePosition;
    private StatusSignalValue<Double> m_driveVelocity;
    private StatusSignalValue<Double> m_driveAppliedCurrent;
    private StatusSignalValue<Double> m_driveSuppliedCurrent;
    private StatusSignalValue<Double> m_driveTempCelsius;
    private StatusSignalValue<Double> m_steerPosition;
    private StatusSignalValue<Double> m_steerVelocity;
    private StatusSignalValue<Double> m_steerAppliedCurrent;
    private StatusSignalValue<Double> m_steerSuppliedCurrent;
    private StatusSignalValue<Double> m_steerTempCelsius;
    private BaseStatusSignalValue[] m_signals;

    private SwerveModuleState m_state = new SwerveModuleState();
    private SwerveModulePosition m_position = new SwerveModulePosition();

    public FalconSwerveIO(int moduleID, String canbus) {
        m_drive = new TalonFX(10 + moduleID, canbus);
        m_steer = new TalonFX(20 + moduleID, canbus);
        m_encoder = new CANcoder(20 + moduleID, canbus);

        m_driveControl = new VelocityTorqueCurrentFOC(0, 0, 0, false);

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        m_drive.getConfigurator().apply(driveConfig);
        driveHelper = new FalconFeedbackControlHelper(m_drive, null, null);


        m_steerControl = new PositionTorqueCurrentFOC(0, 0, 0, false);

        // steerConfig.Slot0 = TODO: PID gains
        steerConfig.TorqueCurrent.PeakForwardTorqueCurrent = 20;
        steerConfig.TorqueCurrent.PeakReverseTorqueCurrent = -20;
        steerConfig.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.RotorToSensorRatio = Constants.kSteerReduction;
        m_steer.getConfigurator().apply(steerConfig);
        steerHelper = new FalconFeedbackControlHelper(m_steer, null, null);

        m_encoder.getConfigurator().refresh(encoderConfig);
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        m_encoder.getConfigurator().apply(encoderConfig);

        m_drivePosition = m_drive.getPosition();
        m_driveVelocity = m_drive.getVelocity();
        m_driveAppliedCurrent = m_drive.getTorqueCurrent();
        m_driveSuppliedCurrent = m_drive.getSupplyCurrent();
        m_driveTempCelsius = m_drive.getDeviceTemp();
        m_steerPosition = m_steer.getPosition();
        m_steerVelocity = m_steer.getVelocity();
        m_steerAppliedCurrent = m_steer.getTorqueCurrent();
        m_steerSuppliedCurrent = m_steer.getSupplyCurrent();
        m_steerTempCelsius = m_steer.getDeviceTemp();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        m_drivePosition.refresh();
        m_driveVelocity.refresh();
        m_driveAppliedCurrent.refresh();
        m_driveSuppliedCurrent.refresh();
        m_driveTempCelsius.refresh();
        m_steerPosition.refresh();
        m_steerVelocity.refresh();
        m_steerAppliedCurrent.refresh();
        m_steerSuppliedCurrent.refresh();
        m_steerTempCelsius.refresh();
        
        double position_compensated = m_drivePosition.getValue() + (m_driveVelocity.getValue() * m_drivePosition.getTimestamp().getLatency());
        double angle_compensated = m_steerPosition.getValue() + (m_steerVelocity.getValue() * m_steerPosition.getTimestamp().getLatency());

        inputs.driveMeters = convertRotationsToMeters(position_compensated);
        inputs.driveVelocityMetersPerSec = convertRotationsToMeters(m_driveVelocity.getValue());
        inputs.driveAppliedCurrentAmps = m_driveAppliedCurrent.getValue();
        inputs.driveSuppliedCurrentAmps = m_driveSuppliedCurrent.getValue();
        inputs.driveTempCelsius = m_driveTempCelsius.getValue();

        inputs.steerPositionRotations = angle_compensated;
        inputs.steerVelocityRotPerSec = m_steerVelocity.getValue();
        inputs.steerAppliedCurrentAmps = m_steerAppliedCurrent.getValue();
        inputs.steerSuppliedCurrentAmps = m_steerSuppliedCurrent.getValue();
        inputs.steerTempCelsius = m_steerTempCelsius.getValue();
    }

    @Override
    public void updateOutputs() {
        m_drive.setControl(m_driveControl);
        m_steer.setControl(m_steerControl);
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
    public void setDriveBrakeMode(boolean driveBrakeMode) {
        TalonFXLiveConfigHelper.editConfig(m_drive, (c) -> {
            c.MotorOutput.NeutralMode = (driveBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            return c;
        });
    }

    @Override
    public void setSteerBrakeMode(boolean steerBrakeMode) {
        TalonFXLiveConfigHelper.editConfig(m_steer, (c) -> {
            c.MotorOutput.NeutralMode = (steerBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
            return c;
        });
    }

    @Override
    public void setDriveKP(double driveKP) {
        driveHelper.setKP(driveKP);
    }

    @Override
    public void setDriveKI(double driveKI) {
        driveHelper.setKI(driveKI);
    }

    @Override
    public void setDriveKD(double drivekD) {
        driveHelper.setKD(drivekD);
    }

    @Override
    public void setDriveKF(double drivekF) {
        driveHelper.setKV(drivekF);
    }

    @Override
    public void setSteerKP(double steerKP) {
        steerHelper.setKP(steerKP);
    }

    @Override
    public void setSteerKI(double steerKI) {
        steerHelper.setKI(steerKI);
    }

    @Override
    public void setSteerKD(double steerKD) {
        steerHelper.setKD(steerKD);
    }

    @Override
    public void setSteerKF(double steerKF) {
        steerHelper.setKV(steerKF);
    }

    @Override
    public void updateEncoderOffset(double zeroRotations) {
        CANcoderLiveConfigHelper.editConfig(m_encoder, (c) -> {
            c.MagnetSensor.MagnetOffset = zeroRotations;
            return c;
        });
    }

    @Override
    public double getEncoderRawPosition() {
        m_encoder.getConfigurator().refresh(encoderConfig);
        var currentOffset = encoderConfig.MagnetSensor.MagnetOffset;
        return m_encoder.getAbsolutePosition().getValue() - currentOffset;
    }
}
