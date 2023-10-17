// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.function.Supplier;

/** Add your docs here. */
public class WidgetDataSource<T> {
    private final Supplier<T> supplier;

    public WidgetDataSource(Supplier<T> supplier) {
        this.supplier = supplier;
    }
}
