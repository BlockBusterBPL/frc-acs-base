// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class GripperMiniNeoSimIO implements GripperIO {
    private final DCMotor cube;
    private final DCMotor cone;

    private double cubeThrottle;
    private double coneThrottle;

    public GripperMiniNeoSimIO() {
        cube = DCMotor.getNeo550(1);
        cone = DCMotor.getNeo550(1);
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        //TODO gripper current sim
    }
}
