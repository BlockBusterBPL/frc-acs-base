package frc.robot.lib.geometry;

public interface IRotation2d<S> extends State<S> {
    ImprovedRotation2d getRotation();

    S rotateBy(ImprovedRotation2d other);

    S mirror();
}
