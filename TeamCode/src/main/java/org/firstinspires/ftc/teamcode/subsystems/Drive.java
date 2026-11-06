
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {
    private DcMotorEx fl;
    private DcMotorEx fr;
    private DcMotorEx bl;
    private DcMotorEx br;

    private double kP = 0.05;
    private double kI = 0;
    private double kD = 0;

    private double kPHeading = 0.5;
    private double kIHeading = 0;
    private double kDHeading = 0;

    private double integralX = 0;
    private double integralY = 0;
    private double integralHeading = 0;

    private double lastErrorX = 0;
    private double lastErrorY = 0;
    private double lastErrorHeading = 0;

    private long lastTime = System.nanoTime();

    public Drive(HardwareMap hardwareMap) {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");

        fl.setDirection(DcMotorEx.Direction.REVERSE);
        bl.setDirection(DcMotorEx.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double turn, double power) {
        double flPower = -forward - strafe - turn;
        double frPower = -forward + strafe + turn;
        double blPower = -forward + strafe - turn;
        double brPower = -forward - strafe + turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                        Math.max(Math.abs(frPower),
                                Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max * power);
        fr.setPower(frPower / max * power);
        bl.setPower(blPower / max * power);
        br.setPower(brPower / max * power);
    }
    public void drive(double forward, double strafe, double turn) {
        double flPower = -forward - strafe - turn;
        double frPower = -forward + strafe + turn;
        double blPower = -forward + strafe - turn;
        double brPower = -forward - strafe + turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(flPower),
                        Math.max(Math.abs(frPower),
                                Math.max(Math.abs(blPower), Math.abs(brPower)))));

        fl.setPower(flPower / max);
        fr.setPower(frPower / max);
        bl.setPower(blPower / max);
        br.setPower(brPower / max);
    }

    public void goToPoint(double targetX, double targetY,
                          double currentX, double currentY,
                          double heading, double targetHeading) {

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;

        double distance = Math.hypot(errorX, errorY);

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        if (dt <= 0) return;

        // Position PID
        double fieldX = 0;
        double fieldY = 0;

        if (distance >= 1.0) {
            integralX += errorX * dt;
            integralY += errorY * dt;

            double derivativeX = (errorX - lastErrorX) / dt;
            double derivativeY = (errorY - lastErrorY) / dt;

            fieldX = kP * errorX
                    + kI * integralX
                    + kD * derivativeX;

            fieldY = kP * errorY
                    + kI * integralY
                    + kD * derivativeY;
        }

        lastErrorX = errorX;
        lastErrorY = errorY;

        // Convert field coordinates to robot coordinates
        double strafe = fieldX * Math.cos(heading)
                + fieldY * Math.sin(heading);

        double forward = -fieldX * Math.sin(heading)
                + fieldY * Math.cos(heading);

        // Heading PID
        double errorHeading = Math.atan2(
                Math.sin(targetHeading - heading),
                Math.cos(targetHeading - heading)
        );

        integralHeading += errorHeading * dt;

        double derivativeHeading =
                (errorHeading - lastErrorHeading) / dt;

        double turn = kPHeading * errorHeading
                + kIHeading * integralHeading
                + kDHeading * derivativeHeading;

        lastErrorHeading = errorHeading;

        drive(forward, strafe, turn);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public void resetPID() {
        integralX = 0;
        integralY = 0;
        integralHeading = 0;

        lastErrorX = 0;
        lastErrorY = 0;
        lastErrorHeading = 0;

        lastTime = System.nanoTime();
    }
}
