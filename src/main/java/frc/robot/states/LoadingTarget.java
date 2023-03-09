package frc.robot.states;

public class LoadingTarget {
    public static enum LOADING_LOCATION {
        GROUND,
        DOUBLE_SUBSTATION_OUTER,
        DOUBLE_SUBSTATION_INNER
    }

    private LOADING_LOCATION loadingLocation;


    public LoadingTarget(LOADING_LOCATION loadingLocation) {
        this.loadingLocation = loadingLocation;
    }

    public LOADING_LOCATION getLoadingLocation() {
        return this.loadingLocation;
    }

    public void setLoadingLocation(LOADING_LOCATION loadingLocation) {
        this.loadingLocation = loadingLocation;
    }
}
