// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** A generic wrapper for tuning feedback control constants */
public class TunablePID{
    private final String sourceType;

    private final DoubleConsumer setKP;
    private final DoubleConsumer setKI;
    private final DoubleConsumer setKD;
    private final DoubleConsumer setKF;

    private final DoubleConsumer setTV;
    private final DoubleConsumer setTA;
    private final DoubleConsumer setTJ;

    private final DoubleSupplier getKP;
    private final DoubleSupplier getKI;
    private final DoubleSupplier getKD;
    private final DoubleSupplier getKF;

    private final DoubleSupplier getTV;
    private final DoubleSupplier getTA;
    private final DoubleSupplier getTJ;

    public TunablePID(ProfiledPIDController controller) {
        sourceType ="ProfiledPIDController (WPI)";

        setKP = (kP) -> controller.setP(kP);
        setKI = (kI) -> controller.setI(kI);
        setKD = (kD) -> controller.setD(kD);
        setKF = (kF) -> {};

        setTV = (tV) -> {};
        setTA = (tA) -> {};
        setTJ = (tJ) -> {};

        getKP = () -> controller.getP();
        getKI = () -> controller.getI();
        getKD = () -> controller.getD();
        getKF = () -> 0.0;

        getTV = () -> 0.0;
        getTA = () -> 0.0;
        getTJ = () -> 0.0;
    }

    public TunablePID(PIDController controller) {
        sourceType ="PIDController (WPI)";

        setKP = (kP) -> controller.setP(kP);
        setKI = (kI) -> controller.setI(kI);
        setKD = (kD) -> controller.setD(kD);
        setKF = (kF) -> {};

        setTV = (tV) -> {};
        setTA = (tA) -> {};
        setTJ = (tJ) -> {};

        getKP = () -> controller.getP();
        getKI = () -> controller.getI();
        getKD = () -> controller.getD();
        getKF = () -> 0.0;

        getTV = () -> 0.0;
        getTA = () -> 0.0;
        getTJ = () -> 0.0;
    }

    public TunablePID(TalonFX talon) {
        sourceType ="TalonFX (Pro)";

        FalconFeedbackControlHelper helper = new FalconFeedbackControlHelper(talon);

        setKP = (kP) -> helper.setKP(kP);
        setKI = (kI) -> helper.setKI(kI);
        setKD = (kD) -> helper.setKD(kD);
        setKF = (kF) -> helper.setKV(kF);

        setTV = (tV) -> helper.setMagicVelocity(tV);
        setTA = (tA) -> helper.setMagicAcceleration(tA);
        setTJ = (tJ) -> helper.setMagicJerk(tJ);

        getKP = () -> helper.getKP();
        getKI = () -> helper.getKI();
        getKD = () -> helper.getKD();
        getKF = () -> helper.getKV();

        getTV = () -> helper.getMagicVelocity();
        getTA = () -> helper.getMagicAcceleration();
        getTJ = () -> helper.getMagicJerk();
    }

    public void setKP(double kP) {
        setKP.accept(kP);
    }
}
