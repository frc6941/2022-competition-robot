package frc.robot.utils;

import org.frcteam6941.utils.ColorConversions;

import edu.wpi.first.wpilibj.util.Color;

/**
 * From Team 254.
 */
public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }
    }

    class RainbowLEDState implements TimedLEDState{
        double mCycleTime;

        public RainbowLEDState(double cycleTime) {
            mCycleTime = cycleTime;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            double hue = (timestamp % mCycleTime) / mCycleTime * 180.0;
            desiredState.copyFrom(LEDState.createFromHSV(hue, 255, 255));
        }
    }

    class BreathingLEDState implements TimedLEDState{
        double mCycleTime;
        double mMaxValue;
        double h;
        double s;

        public BreathingLEDState(LEDState state, double cycleTime) {
            mCycleTime = cycleTime;
            double[] hsvTarget = ColorConversions.fromRGBtoHSV(new Color(state.red, state.green, state.blue));
            h = hsvTarget[0];
            s = hsvTarget[1];
            mMaxValue = hsvTarget[2];
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            double value = Math.abs(Math.sin(Math.PI / mCycleTime * (timestamp % mCycleTime)));
            desiredState.copyFrom(LEDState.createFromHSV(h, s, value));
        }
    }
}