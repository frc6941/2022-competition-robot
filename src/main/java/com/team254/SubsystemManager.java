package com.team254;

import com.team254.loops.ILooper;
import com.team254.loops.Loop;
import com.team254.loops.Looper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
    public static SubsystemManager mInstance = null;

    private List<Subsystem254> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }

        return mInstance;
    }

    /**
     * this method calls "outputTelemetry" for all subsystems. It needs to be called seperately, preferably in RobotPeriodic.
     */
    public void outputToSmartDashboard() {
        mAllSubsystems.forEach(Subsystem254::outputTelemetry);
    }


    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem254 s : mAllSubsystems) {
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public void stop() {
        mAllSubsystems.forEach(Subsystem254::stop);
    }

    public List<Subsystem254> getSubsystems() {
        return mAllSubsystems;
    }

    public void setSubsystems(Subsystem254... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }

    //readPeriodic Inputs would be called always, writePeriodic outputs only when enabled.
    private class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {
            mAllSubsystems.forEach(Subsystem254::readPeriodicInputs);
            mLoops.forEach(l -> l.onLoop(timestamp));
            mAllSubsystems.forEach(Subsystem254::writePeriodicOutputs);
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            mAllSubsystems.forEach(Subsystem254::readPeriodicInputs);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}
