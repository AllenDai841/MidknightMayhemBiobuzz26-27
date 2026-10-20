package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WestCoastDrive;

@TeleOp
public class WestCoastTeleOp extends OpMode {
    public static double DEADZONE = 0.05;
    public static double SLOW_SPEED = 0.4;

    WestCoastDrive drive;

    @Override
    public void init() {
        drive = new WestCoastDrive(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = shape(-gamepad1.left_stick_y);
        double turn = shape(gamepad1.right_stick_x);
        double speed = gamepad1.right_bumper ? SLOW_SPEED : 1.0;
        drive.drive(forward * speed, turn * speed);

        telemetry.addData("foward", "%.2f", forward * speed);
        telemetry.addData("turn", "%.2f", turn * speed);
        telemetry.addData("slow mode", gamepad1.right_bumper);
    }
    private double shape(double stick) {
        if (Math.abs(stick) < DEADZONE) return 0;
        return Math.copySign(stick * stick, stick);
    }
}
