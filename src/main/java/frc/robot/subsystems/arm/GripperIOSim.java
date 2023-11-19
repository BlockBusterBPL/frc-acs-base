// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

/** Add your docs here. */
public class GripperIOSim implements GripperIO {
    private static final LoggedDashboardBoolean fakeCubeSensorTriggered = new LoggedDashboardBoolean("SimControls/Gripper/CubeSensor", false);
    private static final LoggedDashboardBoolean fakeConeSensorTriggered = new LoggedDashboardBoolean("SimControls/Gripper/ConeSensor", false);

    public GripperIOSim() {

    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        inputs.cubeInIntake = fakeCubeSensorTriggered.get();
        inputs.coneInIntake = fakeConeSensorTriggered.get();
    }
}
