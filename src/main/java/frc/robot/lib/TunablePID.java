// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/** A generic wrapper for tuning feedback control constants */
public class TunablePID {
    private final String sourceType;

    private final Consumer<Double> setKP;
    private final Consumer<Double> setKI;
    private final Consumer<Double> setKD;
    private final Consumer<Double> setKF;

    private final Consumer<Double> setTV;
    private final Consumer<Double> setTA;
    private final Consumer<Double> setTJ;

    private final Supplier<Double> getKP;
    private final Supplier<Double> getKI;
    private final Supplier<Double> getKD;
    private final Supplier<Double> getKF;

    private final Supplier<Double> getTV;
    private final Supplier<Double> getTA;
    private final Supplier<Double> getTJ;

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
