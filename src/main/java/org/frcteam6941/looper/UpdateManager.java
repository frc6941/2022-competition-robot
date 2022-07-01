package org.frcteam6941.looper;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.team254.lib.util.CrashTrackingRunnable;

public final class UpdateManager {
	private final Object taskRunningLock_ = new Object();
	public final List<Updatable> updatables = new ArrayList<>();

	public interface Updatable {
		void read(double time, double dt);
		void update(double time, double dt);
		void write(double time, double dt);
		void telemetry();
		void stop();
		void disabled(double time, double dt);
	}

	private double lastTimestamp = 0.0;

	private CrashTrackingRunnable enableRunnable = new CrashTrackingRunnable() {
		@Override
        public void runCrashTracked() {
            synchronized (taskRunningLock_) {
				final double timestamp = Timer.getFPGATimestamp();
				final double dt = timestamp - lastTimestamp;
				lastTimestamp = timestamp;
				updatables.forEach(s -> {
					s.read(timestamp, dt);
					s.update(timestamp, dt);
					s.write(timestamp, dt);
					s.telemetry();
				});
            }
        }
	};

	private CrashTrackingRunnable disableRunnable = new CrashTrackingRunnable() {
		@Override
        public void runCrashTracked() {
            synchronized (taskRunningLock_) {
				final double timestamp = Timer.getFPGATimestamp();
				final double dt = timestamp - lastTimestamp;
				lastTimestamp = timestamp;
				updatables.forEach(s -> {
					s.disabled(timestamp, dt);
					s.read(timestamp, dt);
					s.update(timestamp, dt);
					s.telemetry();
				});
            }
        }
	};

	private final Notifier updaterEnableThread = new Notifier(enableRunnable);
	private final Notifier updaterDisableThread = new Notifier(disableRunnable);

	public UpdateManager(Updatable... updatables) {
		this(Arrays.asList(updatables));
	}

	public UpdateManager(List<Updatable> updatables) {
		this.updatables.addAll(updatables);
	}

	public void startEnableLoop(double period) {
		updaterEnableThread.startPeriodic(period);
	}

	public void stopEnableLoop() {
		updaterEnableThread.stop();
		updatables.forEach(s -> s.stop());
	}

	public void startDisableLoop(double period) {
		updaterDisableThread.startPeriodic(period);
	}

	public void stopDisableLoop() {
		updaterDisableThread.stop();
	}
}
