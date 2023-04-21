package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.ImprovedRotation2d;
import frc.robot.lib.geometry.Translation2d;
import frc.robot.lib.geometry.ImprovedTwist2d;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;
import frc.robot.lib.util.MovingAverageTwist2d;
import frc.robot.lib.util.Util;

import java.util.List;
import java.util.Map;
import java.util.Optional;

public class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;

    /*
     * RobotState keeps track of the poses of various coordinate frames throughout
     * the match. A coordinate frame is simply a point and direction in space that
     * defines an (x,y) coordinate system. Transforms (or poses) keep track of the
     * spatial relationship between different frames.
     *
     * Robot frames of interest (from parent to child):
     *
     * 1. Field frame: origin is where the robot is turned on.
     *
     * 2. Vehicle frame: origin is the center of the robot wheelbase, facing
     * forwards
     *
     * 3. Turret frame: origin is the center of the turret.
     *
     * 4. Camera frame: origin is the center of the Limelight relative to the
     * turret.
     *
     * 5. Target frame: origin is the center of the vision target, facing outwards
     * along the normal.
     *
     * As a kinematic chain with 5 frames, there are 4 transforms of interest:
     *
     * 1. Field-to-vehicle: This is tracked over time by integrating encoder and
     * gyro measurements. It will inevitably drift, but is usually accurate over
     * short time periods.
     *
     * 2. Vehicle-to-turret: Rotation measured by the turret encoder; translation is constant.
     *
     * 3. Turret-to-camera: This is a constant (per camera).
     *
     * 4. Camera-to-target: Measured by the vision system.
     */

    // FPGATimestamp -> Pose2d or Rotation2d


    private Translation2d camera_to_goal_ = Translation2d.identity();

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> vehicle_to_turret_;

    private ImprovedTwist2d vehicle_velocity_predicted_;
    private ImprovedTwist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;

    // In deg/s
    private double turret_velocity_measured_;

    public ImprovedRotation2d prev_heading_;

    private RobotState() {
        reset(0.0, Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle,
                                   Pose2d initial_vehicle_to_turret) {
        reset(start_time, initial_field_to_vehicle);
        vehicle_to_turret_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        vehicle_to_turret_.put(new InterpolatingDouble(start_time), initial_vehicle_to_turret);
    }

    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle) {
        field_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_vehicle_.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        vehicle_velocity_predicted_ = ImprovedTwist2d.identity();
        vehicle_velocity_measured_ = ImprovedTwist2d.identity();
        vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity(), Pose2d.identity());
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getVehicleToTurret(double timestamp) {
        return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Pose2d getFieldToTurret(double timestamp) {
        return getFieldToVehicle(timestamp).transformBy(getVehicleToTurret(timestamp));
    }

    // public synchronized Optional<Pose2d> getVehicleToGoal(double timestamp) {
    //     Pose2d fieldToGoal = getFieldToGoal();

    //     if (fieldToGoal == null) {
    //         return Optional.empty();
    //     }

    //     return Optional.of(getFieldToVehicle(timestamp).inverse().transformBy(fieldToGoal));
    // }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle() {
        return field_to_vehicle_.lastEntry();
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestVehicleToTurret() {
        return vehicle_to_turret_.lastEntry();
    }

    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time) {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(vehicle_velocity_predicted_.scaled(lookahead_time)));
    }

    public synchronized void addVehicleToTurretObservation(double timestamp, Pose2d observation,
                                                           double vel_deg_s) {
        vehicle_to_turret_.put(new InterpolatingDouble(timestamp), observation);
        turret_velocity_measured_ = vel_deg_s;
    }

    public synchronized void addFieldToVehicleObservation(double timestamp, Pose2d observation) {
        field_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addObservations(double timestamp, Pose2d field_to_robot, ImprovedTwist2d measured_velocity, ImprovedTwist2d predicted_velocity) {
        addFieldToVehicleObservation(timestamp, field_to_robot);

        vehicle_velocity_measured_ = measured_velocity;
        vehicle_velocity_measured_filtered_.add(vehicle_velocity_measured_);
        vehicle_velocity_predicted_ = predicted_velocity;
    }

    public synchronized ImprovedTwist2d getPredictedVelocity() {
        return vehicle_velocity_predicted_;
    }

    public synchronized ImprovedTwist2d getMeasuredVelocity() {
        return vehicle_velocity_measured_;
    }

    public synchronized double getMeasuredTurretVelocity() {
        return turret_velocity_measured_;
    }

    public synchronized ImprovedTwist2d getSmoothedVelocity() {
        return vehicle_velocity_measured_filtered_.getAverage();
    }

    public synchronized void resetVision() {
        camera_to_goal_ = Translation2d.identity();
    }

    // public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
    //     if (observations == null || observations.isEmpty()) {
    //         goal_tracker_.maybePruneTracks();
    //         return;
    //     }
    //     updateGoalTracker(timestamp, getCameraToGoalTranslation(observations.get(0), source), goal_tracker_, source);
    // }

    public ImprovedRotation2d getCameraToTargetRotation() {
        return new ImprovedRotation2d(camera_to_goal_, true);

    }

    // private Translation2d getCameraToGoalTranslation(TargetInfo target, Limelight source) {
    //     camera_to_goal_ = getCameraToGoalTranslation(target, source.getLensHeight(), source.getHorizontalPlaneToLens());
    //     return camera_to_goal_;
    // }

    // public static Translation2d getCameraToGoalTranslation(TargetInfo target, double cameraHeight, Rotation2d cameraPitch) {
    //     // Compensate for camera pitch
    //     Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(cameraPitch);
    //     double x = xz_plane_translation.x();
    //     double y = target.getY();
    //     double z = xz_plane_translation.y();

    //     // find intersection with the goal
    //     double differential_height = Constants.kVisionTargetHeight - cameraHeight;
    //     if ((z > 0.0) == (differential_height > 0.0)) {
    //         double scaling = differential_height / z;
    //         double distance = Math.hypot(x, y) * scaling + Constants.kVisionTargetToGoalCenter;
    //         Rotation2d angle = new Rotation2d(x, y, true);
    //         return new Translation2d(distance * angle.cos(), distance * angle.sin());
    //     }
    //     return null;
    // }

    // private void updateGoalTracker(double timestamp, Translation2d cameraToGoalTranslation, GoalTracker tracker, Limelight source) {
    //     if (cameraToGoalTranslation == null) {
    //         return;
    //     }

    //     Pose2d cameraToGoal = Pose2d.fromTranslation(cameraToGoalTranslation);

    //     Pose2d fieldToGoal = getFieldToTurret(timestamp).transformBy(source.getTurretToLens()).transformBy(cameraToGoal);

    //     // Goal normal is always oriented at 180 deg.
    //     tracker.update(timestamp, List.of(new Pose2d(fieldToGoal.getTranslation(), Rotation2d.fromDegrees(180.0))));
    // }

    // public synchronized Pose2d getFieldToGoal() {
    //     GoalTracker tracker = goal_tracker_;
    //     if (!tracker.hasTracks()) {
    //         return null;
    //     }
    //     return tracker.getTracks().get(0).field_to_target;
    // }

    
    public Pose2d getRobot() {
        return new Pose2d();
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());

        SmartDashboard.putString("Field To Robot", getLatestFieldToVehicle().getValue().toString());


        // getVehicleToGoal(Timer.getFPGATimestamp()).ifPresent(p ->  SmartDashboard.putString("Robot to Goal", p.toString()));

    }
}