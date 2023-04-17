package frc.robot.lib.trajectory;

import frc.robot.lib.geometry.State;

public interface TrajectoryView<S extends State<S>, T extends State<T>> {
    TrajectorySamplePoint<S, T> sample(final double interpolant);

    double first_interpolant();

    double last_interpolant();

    Trajectory<S, T> trajectory();
}
