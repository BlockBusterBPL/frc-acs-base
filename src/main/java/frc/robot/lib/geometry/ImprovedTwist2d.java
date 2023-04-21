package frc.robot.lib.geometry;

import java.text.DecimalFormat;

import frc.robot.lib.util.Util;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class ImprovedTwist2d {
    protected static final ImprovedTwist2d kIdentity = new ImprovedTwist2d(0.0, 0.0, 0.0);

    public static ImprovedTwist2d identity() {
        return kIdentity;
    }

    public double dx;
    public double dy;
    public double dtheta; // Radians!

    public ImprovedTwist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    public ImprovedTwist2d scaled(double scale) {
        return new ImprovedTwist2d(dx * scale, dy * scale, dtheta * scale);
    }

    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }

    public boolean epsilonEquals(final ImprovedTwist2d other, double epsilon) {
        return Util.epsilonEquals(dx, other.dx, epsilon) &&
               Util.epsilonEquals(dy, other.dy, epsilon) &&
               Util.epsilonEquals(dtheta, other.dtheta, epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }
}