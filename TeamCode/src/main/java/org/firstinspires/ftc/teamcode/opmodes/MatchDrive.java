package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drive;


@TeleOp
public class MatchDrive extends OpMode {

    Drive drive;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper) {
            drive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    0.5
            );
            telemetry.addData("slow", 1);
        }
        else{
            drive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    1
            );
            telemetry.addData("fast", 1);
        }

    }
}