package org.frcteam6941.looper;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class UpdateManager {
	private final Object taskRunningLock_ = new Object();
	public final List<Updatable> updatables = new ArrayList<>();

	public interface Updatable {
		void read(double time, double dt);

		void update(double time, double dt);

		void write(double time, double dt);

		void telemetry();

		void start();

		void stop();

		void simulate(double time, double dt);
	}

	private double lastTimestamp = 0.0;

	private Runnable enableRunnable = new Runnable() {
		@Override
		public void run() {
			synchronized (taskRunningLock_) {
				final double timestamp = Timer.getFPGATimestamp();
				final double dt = timestamp - lastTimestamp > 10e-5 ? timestamp - lastTimestamp : Constants.LOOPER_DT;
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

	private Runnable simulationRunnable = new Runnable() {
		@Override
		public void run() {
			synchronized (taskRunningLock_) {
				final double timestamp = Timer.getFPGATimestamp();
				final double dt = timestamp - lastTimestamp > 10e-5 ? timestamp - lastTimestamp : Constants.LOOPER_DT;
				lastTimestamp = timestamp;
				updatables.forEach(s -> {
					s.simulate(timestamp, dt);
					s.update(timestamp, dt);
					s.write(timestamp, dt);
					s.telemetry();
				});

			}
		}
	};

	private final Notifier updaterEnableThread = new Notifier(enableRunnable);
	private final Notifier updaterSimulationThread = new Notifier(simulationRunnable);

	public UpdateManager(Updatable... updatables) {
		this(Arrays.asList(updatables));
	}

	public UpdateManager(List<Updatable> updatables) {
		this.updatables.addAll(updatables);
	}

	public void startEnableLoop(double period) {
		updaterEnableThread.startPeriodic(period);
	}

	public void runEnableSingle() {
		enableRunnable.run();
	}

	public void stopEnableLoop() {
		updaterEnableThread.stop();
	}

	public void startSimulateLoop(double period) {
		updaterSimulationThread.startPeriodic(period);
	}

	public void runSimulateSingle() {
		simulationRunnable.run();
	}

	public void stopSimulateLoop() {
		updaterSimulationThread.stop();
	}

	public void invokeStart() {
		updatables.forEach(s -> s.start());
	}

	public void invokeStop() {
		updatables.forEach(s -> s.stop());
	}
}
