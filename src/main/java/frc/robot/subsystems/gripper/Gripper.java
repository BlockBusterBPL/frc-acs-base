// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.lib.Utility;
import frc.robot.subsystems.arm.Arm;

/** Add your docs here. */
public class Gripper extends SubsystemBase {
    private GripperIO io;
    private GripperIOInputsAutoLogged inputs;

    public Gripper(GripperIO io) {
        this.io = io;
        inputs = new GripperIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Gripper", inputs);

        Robot.updateSimCurrentDraw(this.getClass().getName(), Utility.sum(inputs.motorCurrentAmps));
    }

    public void set(double throttle) {
        boolean isConeMode = Arm.isConeMode();

        io.setConeMode(isConeMode);
        io.setMotor(throttle);
    }
}
