package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

public class GyroPigeon5IO implements GyroIO {
    private final Pigeon2 mPigeon;

    public GyroPigeon5IO(int id, String bus) {
        mPigeon = new Pigeon2(id, bus);
        mPigeon.setYaw(0, 50);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.calibrating = false;
        inputs.connected = true;

        inputs.pitchAngleRotations = mPigeon.getPitch() / 360.0;
        inputs.rollAngleRotations = mPigeon.getRoll() / 360.0;
        inputs.yawAngleRotations = mPigeon.getYaw() / 360.0;
    }
}
