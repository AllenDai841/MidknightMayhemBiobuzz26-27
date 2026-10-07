package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.Odometry;

@TeleOp(name = "Odometry Tuner", group = "Odometry")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "fl");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "bl");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "fr");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "br");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[] {frontLeft, backLeft, frontRight, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        Odometry odometry = new Odometry(hardwareMap);

        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();
        odometry.setPose(0, 0, 0);

        double xPodZero = odometry.getXPodDistance();
        double yPodZero = odometry.getYPodDistance();

        boolean calibrating = false;
        boolean haveCalibration = false;
        double calStartHeading = 0, calStartXPod = 0, calStartYPod = 0;
        double turnedDegrees = 0, xPodLeftOffset = 0, yPodForwardOffset = 0;

        long lastLoopTime = System.nanoTime();

        while (opModeIsActive()) {
            odometry.update();

            // --- Drive (robot-centric mecanum) ---
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double scale = Math.max(1, Math.abs(drive) + Math.abs(strafe) + Math.abs(turn));
            frontLeft.setPower((drive + strafe + turn) / scale);
            backLeft.setPower((drive - strafe + turn) / scale);
            frontRight.setPower((drive - strafe - turn) / scale);
            backRight.setPower((drive + strafe - turn) / scale);

            // --- Reset (not during calibration, since it would throw the measurement off) ---
            if (gamepad1.aWasPressed() && !calibrating) {
                odometry.setPose(0, 0, 0);
                xPodZero = odometry.getXPodDistance();
                yPodZero = odometry.getYPodDistance();
            }

            // --- Pod offset calibration ---
            if (gamepad1.xWasPressed()) {
                calibrating = !calibrating;
                if (calibrating) {
                    calStartHeading = odometry.getUnwrappedHeading();
                    calStartXPod = odometry.getXPodDistance();
                    calStartYPod = odometry.getYPodDistance();
                    haveCalibration = false;
                }
            }
            if (calibrating) {
                double turned = odometry.getUnwrappedHeading() - calStartHeading;
                turnedDegrees = Math.toDegrees(turned);
                if (Math.abs(turnedDegrees) > 90) {
                    xPodLeftOffset = -(odometry.getXPodDistance() - calStartXPod) / turned;
                    yPodForwardOffset = (odometry.getYPodDistance() - calStartYPod) / turned;
                    haveCalibration = true;
                }
            }

            // --- Telemetry ---
            long now = System.nanoTime();
            double loopMs = (now - lastLoopTime) / 1e6;
            lastLoopTime = now;

            telemetry.addData("X", "%.2f in", odometry.getX());
            telemetry.addData("Y", "%.2f in", odometry.getY());
            telemetry.addData("Heading", "%.2f deg", odometry.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("X pod", "%.3f in", odometry.getXPodDistance() - xPodZero);
            telemetry.addData("Y pod", "%.3f in", odometry.getYPodDistance() - yPodZero);
            telemetry.addData("Still (ignoring IMU drift)", odometry.isStill());
            telemetry.addData("Loop time", "%.1f ms", loopMs);
            telemetry.addLine();
            if (calibrating) {
                telemetry.addLine("CALIBRATING: spin in place with the right stick, then press X to stop");
            } else {
                telemetry.addLine("Press X to start pod offset calibration");
            }
            if (calibrating || haveCalibration) {
                telemetry.addData("Turned", "%.1f deg (%.2f turns)", turnedDegrees, turnedDegrees / 360);
            }
            if (haveCalibration) {
                telemetry.addData("X_POD_LEFT_OFFSET", "%.3f", xPodLeftOffset);
                telemetry.addData("Y_POD_FORWARD_OFFSET", "%.3f", yPodForwardOffset);
            }
            telemetry.update();
        }
    }
}
