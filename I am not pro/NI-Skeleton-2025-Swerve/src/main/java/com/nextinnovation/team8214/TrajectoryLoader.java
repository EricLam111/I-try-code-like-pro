package com.nextinnovation.team8214;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.nextinnovation.team8214.util.AllianceValue;
import lombok.Getter;
import lombok.Synchronized;

@Getter
public class TrajectoryLoader {
  private static TrajectoryLoader instance = null;

  public static TrajectoryLoader getInstance() {
    if (instance == null) {
      instance = new TrajectoryLoader();
    }
    return instance;
  }

  private TrajectorySet trajectorySet = null;

  private TrajectoryLoader() {
    lazyLoadTrajectorySet();
  }

  @Synchronized
  public void lazyLoadTrajectorySet() {
    if (trajectorySet == null) {
      System.out.println("Lazy loading all trajectories...");
      trajectorySet = new TrajectorySet();
      System.out.println("Trajectories finished loading!");
    }
  }

  public static final class TrajectorySet {
    public AllianceValue<Trajectory<SwerveSample>> test;

    public TrajectorySet() {
      test = loadTrajectory("test");
    }

    @SuppressWarnings("unchecked")
    private AllianceValue<Trajectory<SwerveSample>> loadTrajectory(String name) {
      var trajectory = Choreo.loadTrajectory(name);
      if (trajectory.isEmpty()) {
        throw new NullPointerException("[TrajectoryLoader]: " + name + ".traj not found");
      }
      System.out.println("[TrajectoryLoader]: " + name + ".traj loaded");

      var blue = (Trajectory<SwerveSample>) trajectory.get();

      return new AllianceValue<>(blue, blue.flipped());
    }
  }
}
