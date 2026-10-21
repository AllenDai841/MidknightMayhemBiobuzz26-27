package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm. robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Motor;

@Configurable
@TeleOp
public class outtakeTest extends OpMode{
    Motor motor;
    public static double rpm = 3900;
    @Override
    public void init() {
        motor = new Motor(hardwareMap.get(DcMotorEx.class, "motorTest"), 28);
        motor.setMode (DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        motor.setRPM(rpm);
        telemetry.addLine(String.valueOf(motor.getVelocity()));
    }
}
