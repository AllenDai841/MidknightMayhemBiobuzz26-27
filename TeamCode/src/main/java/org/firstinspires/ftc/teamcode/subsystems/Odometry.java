package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Odometry {
    public static String X_POD_NAME = "xPod";
    public static String Y_POD_NAME = "yPod";
    public static String IMU_NAME = "imu";
    public static RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_DIAMETER = 32 / 25.4;
    public static double X_POD_MULTIPLIER = 1.0;
    public static double Y_POD_MULTIPLIER = 1.0;
    public static boolean X_POD_REVERSED = false;
    public static boolean Y_POD_REVERSED = false;
    public static double X_POD_LEFT_OFFSET = 0.0;
    public static double Y_POD_FORWARD_OFFSET = 0.0;
    public static double IMU_SCALE = 1.0;
    public static boolean USE_DRIFT_CORRECTION = true;
    public static double STILL_TOLERANCE = 0.005;
    public static double STILL_TIME = 0.5;
    public static double MAX_DRIFT_RATE = 0.5;
    private final DcMotorEx xPod;
    private final DcMotorEx yPod;
    private final IMU imu;
    private double x, y, heading;
    private int lastXTicks, lastYTicks;
    private double lastImuYaw, lastTime;
    private boolean needsBaseline = true;
    private double xPodDistance, yPodDistance;
    private double stillStartX, stillStartY;
    private double stillTimer;
    private double heldTurn;

    public Odometry(HardwareMap hardwareMap) {
        xPod = hardwareMap.get(DcMotorEx.class, X_POD_NAME);
        yPod = hardwareMap.get(DcMotorEx.class, Y_POD_NAME);
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));
    }
    public void update() {
        update(xPod.getCurrentPosition(),
                yPod.getCurrentPosition(),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                System.nanoTime() / 1e9);
    }
    void update(int xTicks, int yTicks, double imuYaw, double time) {
        if (needsBaseline) {
            lastXTicks = xTicks;
            lastYTicks = yTicks;
            lastImuYaw = imuYaw;
            lastTime = time;
            needsBaseline = false;
            return;
        }
        double xPodChange = (xTicks - lastXTicks) * inchesPerTick() * X_POD_MULTIPLIER * (X_POD_REVERSED ? -1 : 1);
        double yPodChange = (yTicks - lastYTicks) * inchesPerTick() * Y_POD_MULTIPLIER * (Y_POD_REVERSED ? -1 : 1);
        double imuTurn = angleWrap(imuYaw - lastImuYaw) * IMU_SCALE;
        double dt = time - lastTime;
        lastXTicks = xTicks;
        lastYTicks = yTicks;
        lastImuYaw = imuYaw;
        lastTime = time;
        xPodDistance += xPodChange;
        yPodDistance += yPodChange;
        double turn = removeDrift(imuTurn, dt);
        double forward = xPodChange + turn * X_POD_LEFT_OFFSET;
        double left = yPodChange - turn * Y_POD_FORWARD_OFFSET;

        double halfTurn = turn / 2;
        double arcFactor = Math.abs(halfTurn) < 1e-9 ? 1 : Math.sin(halfTurn) / halfTurn;
        double midHeading = heading + halfTurn;
        double cos = Math.cos(midHeading);
        double sin = Math.sin(midHeading);
        x += arcFactor * (forward * cos - left * sin);
        y += arcFactor * (forward * sin + left * cos);
        heading += turn;
    }
    private double removeDrift(double imuTurn, double dt) {
        if (!driftCorrectionOn()) {
            double turn = imuTurn + heldTurn;
            heldTurn = 0;
            return turn;
        }

        boolean podsMoved = Math.abs(xPodDistance - stillStartX) > STILL_TOLERANCE
                || Math.abs(yPodDistance - stillStartY) > STILL_TOLERANCE;
        if (podsMoved) {
            double turn = imuTurn + heldTurn;
            heldTurn = 0;
            stillTimer = 0;
            stillStartX = xPodDistance;
            stillStartY = yPodDistance;
            return turn;
        }

        stillTimer += dt;
        if (stillTimer < STILL_TIME) {
            return imuTurn;
        }
        heldTurn = heldTurn * Math.exp(-dt / STILL_TIME) + imuTurn;
        if (Math.abs(heldTurn) > Math.toRadians(MAX_DRIFT_RATE) * STILL_TIME) {
            double turn = heldTurn;
            heldTurn = 0;
            stillTimer = 0;
            return turn;
        }
        return 0;
    }
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        needsBaseline = true;
        heldTurn = 0;
        stillTimer = 0;
        stillStartX = xPodDistance;
        stillStartY = yPodDistance;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getHeading() {
        return angleWrap(heading);
    }
    public double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }
    public double getUnwrappedHeading() {
        return heading;
    }
    public double getXPodDistance() {
        return xPodDistance;
    }
    public double getYPodDistance() {
        return yPodDistance;
    }
    public boolean isStill() {
        return driftCorrectionOn() && stillTimer >= STILL_TIME;
    }

    private static boolean driftCorrectionOn() {
        return USE_DRIFT_CORRECTION && Math.max(Math.abs(X_POD_LEFT_OFFSET), Math.abs(Y_POD_FORWARD_OFFSET)) >= 1.0;
    }

    private static double inchesPerTick() {
        return Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    }
    private static double angleWrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
