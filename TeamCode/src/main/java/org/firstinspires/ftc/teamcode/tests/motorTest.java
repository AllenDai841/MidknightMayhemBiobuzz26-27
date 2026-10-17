package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Motor;

@Configurable
@TeleOp
public class motorTest extends OpMode {

    Motor motor;

    public static double rpm = -500;
    public static double rampRate = 15; // RPM per second

    private double commandedRPM = 0;
    private double lastTime = 0;

    @Override
    public void init() {
        motor = new Motor(
                hardwareMap.get(DcMotorEx.class, "motorTest"),
                28
        );

        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        commandedRPM = 0;
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        double now = getRuntime();
        double dt = Math.min(now - lastTime, 0.1);
        lastTime = now;

        double maxChange = rampRate * dt;
        double difference = rpm - commandedRPM;

        if (Math.abs(difference) <= maxChange) {
            commandedRPM = rpm;
        } else {
            commandedRPM += Math.signum(difference) * maxChange;
        }

        motor.setRPM(commandedRPM);

        telemetry.addData("Target RPM", rpm);
        telemetry.addData("Commanded RPM", commandedRPM);
        telemetry.addData(
                "Measured RPM",
                motor.getVelocity() * 60.0 / 28.0
        );
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}