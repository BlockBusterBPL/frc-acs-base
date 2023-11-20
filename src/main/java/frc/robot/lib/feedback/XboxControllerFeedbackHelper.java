// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.feedback;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/** Add your docs here. */
public class XboxControllerFeedbackHelper {
    private static ArrayList<XboxControllerFeedbackHelper> sFeedbackHelpers = new ArrayList<>();

    public enum FeedbackTypes {
        kCoarse,
        kFine,
        kBoth
    }

    public class FeedbackOutput {
        public final FeedbackTypes mFeedbackType;
        public final double mFeedbackStrength;

        public FeedbackOutput(FeedbackTypes type, double strength) {
            mFeedbackType = type;
            mFeedbackStrength = strength;
        }
    }

    public interface FeedbackPattern {
        public FeedbackOutput getOutput(double timeSinceStart);
    }

    public class ConstantFeedback implements FeedbackPattern {
        private final FeedbackOutput mOutput;

        public ConstantFeedback(FeedbackOutput output) {
            mOutput = output;
        }

        @Override
        public FeedbackOutput getOutput(double timeSinceStart) {
            return mOutput;
        }
    }

    public class TimedFeedback implements FeedbackPattern {
        private final double mTimeOn;
        private final double mTimeOff;
        private final FeedbackOutput mOutputWhenOn;
        private final FeedbackOutput mOutputWhenOff;

        public TimedFeedback(double timeOn, double timeOff, FeedbackOutput outputWhenOn, FeedbackOutput outputWhenOff) {
            mTimeOn = timeOn;
            mTimeOff = timeOff;
            mOutputWhenOn = outputWhenOn;
            mOutputWhenOff = outputWhenOff;
        }

        public final FeedbackOutput getOutput(double timeSincePatternStart) {
            if(mTimeOn != mTimeOff) {
                if (timeSincePatternStart % (mTimeOn + mTimeOff) > mTimeOn) {
                    return mOutputWhenOn;
                } else {
                    return mOutputWhenOff;
                }
            } else {
                if ((int) (timeSincePatternStart / mTimeOff) % 2 == 0) {
                    return mOutputWhenOn;
                } else {
                    return mOutputWhenOff;
                }
            }
        }
    }

    public static final void updateAll() {
        sFeedbackHelpers.forEach((h) -> h.update());
    }

    private final XboxController mController;
    private double mPatternStarted;
    private FeedbackPattern mCurrentPattern;

    public XboxControllerFeedbackHelper(XboxController controller) {
        mController = controller;

        sFeedbackHelpers.add(this);
    }

    public void setPattern(FeedbackPattern pattern) {
        mCurrentPattern = pattern;
        mPatternStarted = Timer.getFPGATimestamp();
    }

    public void update() {
        double now = Timer.getFPGATimestamp();
        double timeSinceStart = now - mPatternStarted;
        
        setOutput(mCurrentPattern.getOutput(timeSinceStart));
    }

    public void setOutput(FeedbackOutput output) {
        RumbleType rumbleType = RumbleType.kBothRumble;

        switch (output.mFeedbackType) {
            case kBoth:
                rumbleType = RumbleType.kBothRumble;
                break;
            case kCoarse:
                rumbleType = RumbleType.kLeftRumble;
                break;
            case kFine:
                rumbleType = RumbleType.kRightRumble;
                break;
            default:
                DriverStation.reportWarning("GenericHID Rumble Type conversion fell through.", false);
                break;
        }

        mController.setRumble(rumbleType, output.mFeedbackStrength);
    }

    public void stopOutput() {

    }
}
