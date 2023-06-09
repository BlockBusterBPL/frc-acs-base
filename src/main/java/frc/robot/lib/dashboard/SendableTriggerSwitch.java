// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.dashboard;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.event.EventLoop;

/** Add your docs here. */
public class SendableTriggerSwitch implements Sendable, BooleanSupplier {
    private boolean value;
    private boolean oldValue;
    public final EventLoop anyEdge;
    public final EventLoop risingEdge;
    public final EventLoop fallingEdge;

    private final ArrayList<BooleanConsumer> m_consumers;

    public SendableTriggerSwitch() {
        this.value = false;
        this.anyEdge = new EventLoop();
        this.risingEdge = new EventLoop();
        this.fallingEdge = new EventLoop();

        m_consumers = new ArrayList<>();
        anyEdge.bind(() -> m_consumers.forEach((c) -> c.accept(getAsBoolean())));
    }

    public void bind(BooleanConsumer... consumers) {
        for (BooleanConsumer consumer : consumers) {
            m_consumers.add(consumer);
        }
    }

    public boolean getValue() {
        return value;
    }

    @Override
    public boolean getAsBoolean() {
        return getValue();
    }

    public void setValue(boolean value) {
        this.oldValue = this.value;
        this.value = value;

        if (this.value && !this.oldValue) {
            // rising edge
            anyEdge.poll();
            risingEdge.poll();
        }
        if (!this.value && this.oldValue) {
            // falling edge
            anyEdge.poll();
            fallingEdge.poll();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Value", this::getValue, this::setValue);        
    }
}
