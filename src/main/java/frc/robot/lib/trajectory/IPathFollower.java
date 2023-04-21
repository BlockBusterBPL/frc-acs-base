package frc.robot.lib.trajectory;

import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.ImprovedTwist2d;

public interface IPathFollower {
    ImprovedTwist2d steer(Pose2d current_pose);

    boolean isDone();
}
