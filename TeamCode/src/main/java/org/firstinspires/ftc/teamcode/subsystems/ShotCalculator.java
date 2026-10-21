package org.firstinspires.ftc.teamcode.subsystems;

public class ShotCalculator {
    public enum Hive { RED, BLUE }
    public enum Side { FRONT, BACK }
    public static double HOOD_ANGLE = 45;              // launch angle above horizontal
    public static double SHOOTER_HEIGHT = 12;          // height the ball leaves the shooter
    public static double WHEEL_DIAMETER = 4;           // shooter wheel
    public static double SPEED_EFFICIENCY = 0.5;       // ball speed / wheel surface speed
    public static double MAX_RPM = 5400;
    public static double MIN_DISTANCE = 12;            // too close to shoot
    public static double CELL_SPACING = 18.8;     // manual 9.6.2
    public static double HIVE_Y_OFFSET = 12;      // ESTIMATE
    public static double CELL_HEIGHT = 48;        // ESTIMATE
    public static double[][][] SHOOTING_ZONES = {};

    public Hive hive = Hive.RED;
    public Side side = Side.FRONT;

    private double distance, rpm, targetHeading, aimError;
    private boolean inZone, reachable, canShoot;

    public void update(Odometry odometry) {
        update(odometry.getX(), odometry.getY(), odometry.getHeading());
    }

    public void update(double x, double y, double headingRadians) {
        double[] target = target();
        double dx = target[0] - x;
        double dy = target[1] - y;
        distance = Math.hypot(dx, dy);

        targetHeading = Math.toDegrees(Math.atan2(dy, dx));
        aimError = wrap(targetHeading - Math.toDegrees(headingRadians));
        double angle = Math.toRadians(HOOD_ANGLE);
        double rise = target[2] - SHOOTER_HEIGHT;
        double drop = distance * Math.tan(angle) - rise;
        if (drop > 0 && distance >= MIN_DISTANCE) {
            double speed = Math.sqrt(386.09 * distance * distance / (2 * Math.pow(Math.cos(angle), 2) * drop));
            rpm = speed / SPEED_EFFICIENCY / (Math.PI * WHEEL_DIAMETER) * 60;
            reachable = rpm <= MAX_RPM;
        } else {
            rpm = 0;          // hood too flat, or robot too close
            reachable = false;
        }

        inZone = SHOOTING_ZONES.length == 0;
        for (double[][] zone : SHOOTING_ZONES) {
            if (inside(zone, x, y)) inZone = true;
        }
        canShoot = inZone && reachable;
    }

    private double[] target() {
        double x = side == Side.FRONT ? -CELL_SPACING / 2 : CELL_SPACING / 2;
        double y = hive == Hive.RED ? HIVE_Y_OFFSET : -HIVE_Y_OFFSET;
        return new double[] {x, y, CELL_HEIGHT};
    }

    private static boolean inside(double[][] zone, double x, double y) {
        boolean in = false;
        for (int i = 0, j = zone.length - 1; i < zone.length; j = i++) {
            if ((zone[i][1] > y) != (zone[j][1] > y)
                    && x < (zone[j][0] - zone[i][0]) * (y - zone[i][1]) / (zone[j][1] - zone[i][1]) + zone[i][0]) {
                in = !in;
            }
        }
        return in;
    }

    private static double wrap(double degrees) {
        return degrees - 360 * Math.floor((degrees + 180) / 360);
    }

    public double getRpm() { return rpm; }
    public double getDistance() { return distance; }
    /** Field heading to face, degrees (0 = +X, positive = left). */
    public double getTargetHeading() { return targetHeading; }
    /** Degrees to turn: positive = turn left. */
    public double getAimError() { return aimError; }
    public boolean isInZone() { return inZone; }
    public boolean isReachable() { return reachable; }
    public boolean canShoot() { return canShoot; }
}
