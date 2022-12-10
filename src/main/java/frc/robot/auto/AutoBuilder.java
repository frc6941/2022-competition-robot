package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import org.frcteam6941.swerve.SJTUSwerveMK5Drivebase;
import org.frcteam6941.swerve.SwerveDrivetrainBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.basics.FollowTrajectoryWithEvents;

public class AutoBuilder {
    protected HashMap<String, Command> eventMap = new HashMap<>();

    public static AutoBuilder getInstance() {
        if (instance == null) {
            instance = new AutoBuilder();
        }
        return instance;
    }

    private AutoBuilder() {

    }

    private static AutoBuilder instance;

    public void bindEventMap(HashMap<String, Command> eventMap) {
        this.eventMap = eventMap;
    }

    /**
     * Wrap an event command, so it can be added to a command group
     *
     * @param eventCommand The event command to wrap
     * @return Wrapped event command
     */
    protected CommandBase wrappedEventCommand(Command eventCommand) {
        return new FunctionalCommand(
                eventCommand::initialize,
                eventCommand::execute,
                eventCommand::end,
                eventCommand::isFinished,
                eventCommand.getRequirements().toArray(new Subsystem[0]));
    }

    /**
     * Create a command group to handle all of the commands at a stop event
     *
     * @param stopEvent The stop event to create the command group for
     * @return Command group for the stop event
     */
    public CommandBase stopEventGroup(PathPlannerTrajectory.StopEvent stopEvent) {
        CommandGroupBase events = new ParallelCommandGroup();

        if (stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.SEQUENTIAL) {
            events = new SequentialCommandGroup();
        } else if (stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE) {
            CommandBase deadline = new InstantCommand();
            if (eventMap.containsKey(stopEvent.names.get(0))) {
                deadline = wrappedEventCommand(eventMap.get(stopEvent.names.get(0)));
            }
            events = new ParallelDeadlineGroup(deadline);
        }

        for (int i = (stopEvent.executionBehavior == PathPlannerTrajectory.StopEvent.ExecutionBehavior.PARALLEL_DEADLINE
                ? 1
                : 0); i < stopEvent.names.size(); i++) {
            String name = stopEvent.names.get(i);
            if (eventMap.containsKey(name)) {
                events.addCommands(wrappedEventCommand(eventMap.get(name)));
            }
        }

        switch (stopEvent.waitBehavior) {
            case BEFORE:
                return new SequentialCommandGroup(new WaitCommand(stopEvent.waitTime), events);
            case AFTER:
                return new SequentialCommandGroup(events, new WaitCommand(stopEvent.waitTime));
            case DEADLINE:
                return new ParallelDeadlineGroup(new WaitCommand(stopEvent.waitTime), events);
            case MINIMUM:
                return new ParallelCommandGroup(new WaitCommand(stopEvent.waitTime), events);
            case NONE:
            default:
                return events;
        }
    }

    /**
     * Create a complete autonomous command group. This will reset the robot pose at
     * the begininng of
     * the first path, follow paths, trigger events during path following, and run
     * commands between
     * paths with stop events.
     *
     * <p>
     * Using this does have its limitations, but it should be good enough for most
     * teams. However,
     * if you want the auto command to function in a different way, you can create
     * your own class that
     * extends BaseAutoBuilder and override existing builder methods to create the
     * command group
     * however you wish.
     *
     * @param trajectory Single trajectory to follow during the auto
     * @return Autonomous command
     */
    public CommandBase fullAuto(SJTUSwerveMK5Drivebase driveBase, PathPlannerTrajectory trajectory, boolean angleLock) {
        return fullAuto(driveBase, new ArrayList<>(List.of(trajectory)), angleLock);
    }

    /**
     * Create a complete autonomous command group. This will reset the robot pose at
     * the begininng of
     * the first path, follow paths, trigger events during path following, and run
     * commands between
     * paths with stop events.
     *
     * <p>
     * Using this does have its limitations, but it should be good enough for most
     * teams. However,
     * if you want the auto command to function in a different way, you can create
     * your own class that
     * extends BaseAutoBuilder and override existing builder methods to create the
     * command group
     * however you wish.
     *
     * @param pathGroup Path group to follow during the auto
     * @return Autonomous command
     */
    public CommandBase fullAuto(SJTUSwerveMK5Drivebase driveBase, ArrayList<PathPlannerTrajectory> pathGroup, boolean angleLock) {
        SequentialCommandGroup group = new SequentialCommandGroup();

        boolean reset = true;
        for (PathPlannerTrajectory traj : pathGroup) {
            group.addCommands(
                stopEventGroup(traj.getStartStopEvent()),
                new FollowTrajectoryWithEvents(driveBase, traj, eventMap, angleLock, reset, true)
            );
            reset = false;
        }

        group.addCommands(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));

        return group;
    }
}
