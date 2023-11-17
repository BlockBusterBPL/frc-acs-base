package frc.robot.lib.dashboard;

import java.util.ArrayList;
import java.util.Collection;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class SupplierWidget<T> {
    private static final Collection<SupplierWidget<?>> list = new ArrayList<>();

    private final Supplier<T> supplier;
    private final GenericEntry entry;

    public SupplierWidget(String tab, String title, T initialValue, Supplier<T> supplier, WidgetConfig config) {
        this.supplier = supplier;

        if (config.getUseBuiltInWidgets()) {
            entry = Shuffleboard.getTab(tab).add(title, initialValue)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getBuiltInWidget())
                .withProperties(config.getProperties())
                .getEntry();
        } else {
            entry = Shuffleboard.getTab(tab).add(title, initialValue)
                .withPosition(config.getColumn(), config.getRow())
                .withSize(config.getWidth(), config.getHeight())
                .withWidget(config.getExternalWidget())
                .withProperties(config.getProperties())
                .getEntry();
        }
        
        list.add(this);
    }

    public void update() {
        entry.setValue(supplier.get());
    }

    public static void updateAll() {
        list.forEach((w) -> w.update());
    }
}
