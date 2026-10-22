package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.Motor;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.ShotCalculator;

@TeleOp(name = "Outtake Test", group = "Tests")
public class OuttakeTest extends OpMode {
    Motor motor;
    Odometry odometry;
    final ShotCalculator shotCalculator = new ShotCalculator();

    public static boolean useShotCalculator = false;
    public static double rpm = 3900;

    @Override
    public void init() {
        motor = new Motor(hardwareMap.get(DcMotorEx.class, "motorTest"), 28);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (useShotCalculator) {
            try {
                odometry = new Odometry(hardwareMap);
            } catch (IllegalArgumentException e) {
                telemetry.addData("Missing odometry hardware", "using manual rpm");
                odometry = null;
            }
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        double targetRpm = rpm;
        boolean calcActive = useShotCalculator && odometry != null;

        if (calcActive) {
            odometry.update();
            shotCalculator.update(odometry);
            if (shotCalculator.canShoot()) {
                targetRpm = shotCalculator.getRpm();
            }
        }

        motor.setRPM(targetRpm);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Velocity", "%.0f", motor.getVelocity());
        if (calcActive) {
            telemetry.addData("Shot RPM", "%.0f", shotCalculator.getRpm());
            telemetry.addData("Shot Distance", "%.1f", shotCalculator.getDistance());
            telemetry.addData("Can Shoot", shotCalculator.canShoot());
        }
    }

    @Override
    public void stop() {
        motor.setRPM(0);
    }
}