package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp
public class servoTest extends OpMode {
    public static double targetDegrees = 90;
    public static double speedDegreesPerSecond = 90;
    public static double servoRangeDegrees = 270;

    Servo servo;
    double currentDegrees;
    double lastTime;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        currentDegrees = targetDegrees;
    }

    @Override
    public void start() {
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) targetDegrees += 10;
        if (gamepad1.dpadDownWasPressed()) targetDegrees -= 10;
        if (gamepad1.dpadRightWasPressed()) speedDegreesPerSecond += 10;
        if (gamepad1.dpadLeftWasPressed()) speedDegreesPerSecond = Math.max(10, speedDegreesPerSecond - 10);
        targetDegrees = Math.max(0, Math.min(servoRangeDegrees, targetDegrees));

        double now = getRuntime();
        double maxStep = speedDegreesPerSecond * (now - lastTime);
        lastTime = now;
        double error = targetDegrees - currentDegrees;
        currentDegrees += Math.max(-maxStep, Math.min(maxStep, error));

        servo.setPosition(currentDegrees / servoRangeDegrees);

        telemetry.addData("target", targetDegrees);
        telemetry.addData("current", currentDegrees);
        telemetry.addData("speed", speedDegreesPerSecond);
        telemetry.addLine("TESTING 123");
    }
}
