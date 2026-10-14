package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Motor;

@Configurable
@TeleOp
public class motorTest extends OpMode {
    public static double rpm = 500;
    public static double ticksPerRevolution = 28;

    Motor motor;
    @Override
    public void init() {
        motor = new Motor(hardwareMap.get(DcMotorEx.class, "motor"), ticksPerRevolution);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) rpm += 100;
        if (gamepad1.dpadDownWasPressed()) rpm -= 100;
        if (gamepad1.bWasPressed()) rpm = 0;

        motor.setRPM(rpm);

        telemetry.addData("target", rpm);
        telemetry.addData("actual", motor.getVelocity() * 60 / ticksPerRevolution);
        telemetry.addLine("Type rpm in Panels, or dpad up/down = +/-100, B = stop");
    }
}
