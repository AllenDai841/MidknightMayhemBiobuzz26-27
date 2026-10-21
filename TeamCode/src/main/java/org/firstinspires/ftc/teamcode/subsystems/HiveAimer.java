package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

public class HiveAimer {
    public static int HIVE_TAG_ID = 1;
    public static double HIVE_X = 60;           // inches, in the same coordinates as Odometry
    public static double HIVE_Y = 0;
    public static double AIM_OFFSET = 0;        // degrees, positive aims left of the hive (camera or shooter not centered)
    public static double TURN_GAIN = 0.02;      // turn power per degree of aim error
    public static double MAX_TURN = 0.5;
    public static double AIM_TOLERANCE = 2;     // degrees
    public static double TAG_TRUST = 0.2;       // 0 to 1, how fast the tag corrects the odometry aim

    AprilTagWebcam camera;
    Odometry odometry;
    ShotCalculator shotCalculator = new ShotCalculator();
    boolean tagSeen;
    double correction;
    double error;
    double turn;

    public HiveAimer(AprilTagWebcam camera, Odometry odometry){
        this.camera = camera;
        this.odometry = odometry;
    }

    public void update(){
        camera.update();
        AprilTagSingleDetection tag = camera.getTagBySpecificID(HIVE_TAG_ID);
        boolean seen = tag != null && tag.ftcPose != null;
        update(odometry.getX(), odometry.getY(), odometry.getHeading(), seen, seen ? tag.ftcPose.bearing : 0);
    }

    void update(double x, double y, double heading, boolean tagSeen, double tagBearing){
        shotCalculator.update(x, y, heading);
        this.tagSeen = tagSeen;
        double calculatedError = shotCalculator.getAimError();
        // When the tag is visible, remember how far off odometry was, and keep using that once the tag is lost.
        if (tagSeen){
            correction += TAG_TRUST * wrap(tagBearing - calculatedError - correction);
        }
        error = wrap(calculatedError + correction + AIM_OFFSET);
        // Drive.drive() turns right for positive turn, and a positive error means the hive is to the left.
        turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, -TURN_GAIN * error));
    }

    private static double wrap(double degrees){
        return degrees - 360 * Math.floor((degrees + 180) / 360);
    }

    public double getTurn(){
        return turn;
    }
    public double getError(){
        return error;
    }
    public double getTargetHeading(){
        return shotCalculator.getTargetHeading();
    }
    public double getShotRpm(){
        return shotCalculator.getRpm();
    }
    public double getShotDistance(){
        return shotCalculator.getDistance();
    }
    public boolean canShoot(){
        return shotCalculator.canShoot();
    }
    public boolean isShotReachable(){
        return shotCalculator.isReachable();
    }
    public boolean isAimed(){
        return Math.abs(error) < AIM_TOLERANCE;
    }
    public boolean isTagSeen(){
        return tagSeen;
    }
}
