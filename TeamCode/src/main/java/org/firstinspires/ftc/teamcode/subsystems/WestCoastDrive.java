package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WestCoastDrive {
    public static String[] LEFT_NAMES = {"fl", "bl"};
    public static String[] RIGHT_NAMES = {"fr", "br"};

    DcMotorEx[] left;
    DcMotorEx[] right;

    public WestCoastDrive(HardwareMap hardwareMap) {
        left = new DcMotorEx[LEFT_NAMES.length];
        right = new DcMotorEx[RIGHT_NAMES.length];
        for (int i = 0; i < left.length; i++) {
            left[i] = hardwareMap.get(DcMotorEx.class, LEFT_NAMES[i]);
            left[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
        for (int i = 0; i < right.length; i++) {
            right[i] = hardwareMap.get(DcMotorEx.class, RIGHT_NAMES[i]);
            right[i].setDirection(DcMotorEx.Direction.REVERSE);
            right[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }

    // forward: + drives forward. turn: + turns right.
    public void drive(double forward, double turn) {
        double leftPower = forward + turn;
        double rightPower = forward - turn;
        double max = Math.max(1.0, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
        setPowers(leftPower / max, rightPower / max);
    }

    public void setPowers(double leftPower, double rightPower) {
        for (DcMotorEx motor : left) motor.setPower(leftPower);
        for (DcMotorEx motor : right) motor.setPower(rightPower);
    }
}
