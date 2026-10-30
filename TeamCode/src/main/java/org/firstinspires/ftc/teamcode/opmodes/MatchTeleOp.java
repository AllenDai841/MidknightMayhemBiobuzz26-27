package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Motor;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.ShotCalculator;
import org.firstinspires.ftc.teamcode.subsystems.TransferSensor;

@Configurable
@TeleOp(name = "Match TeleOp")
public class MatchTeleOp extends OpMode {
    public static double START_X = 56;          // same start as Pathfinding
    public static double START_Y = 8;
    public static double START_HEADING = 90;    // degrees
    public static double SHOOTER_TICKS_PER_REV = 28;
    public static double INTAKE_POWER = 1.0;
    public static double TRIGGER_REST_POSITION = 0.0;
    public static double TRIGGER_FIRE_POSITION = 0.45;
    public static double AIM_GAIN = 0.02;       // turn power per degree of aim error
    public static double MAX_AIM_TURN = 0.5;
    public static double RPM_TOLERANCE = 100;   // shooter must be this close before firing
    public static double SLOW_SPEED = 0.4;

    Drive drive;
    Odometry odometry;
    Motor shooter;
    DcMotorEx intake;
    Servo trigger;
    TransferSensor transfer;
    ShotCalculator shot = new ShotCalculator();

    boolean intakeOn;
    boolean shooterOn;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        odometry = new Odometry(hardwareMap);
        odometry.setPose(START_X, START_Y, Math.toRadians(START_HEADING));
        shooter = new Motor(hardwareMap.get(DcMotorEx.class, "shooter"), SHOOTER_TICKS_PER_REV);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        trigger.setPosition(TRIGGER_REST_POSITION);
        transfer = new TransferSensor(hardwareMap.get(DistanceSensor.class, "transfer"));
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) shot.hive = ShotCalculator.Hive.BLUE;
        if (gamepad1.bWasPressed()) shot.hive = ShotCalculator.Hive.RED;
        telemetry.addData("Hive (X = blue, B = red)", shot.hive);
    }

    @Override
    public void loop() {
        odometry.update();
        transfer.update();
        shot.update(odometry);

        if (gamepad1.xWasPressed()) shot.hive = ShotCalculator.Hive.BLUE;
        if (gamepad1.bWasPressed()) shot.hive = ShotCalculator.Hive.RED;
        if (gamepad1.dpadUpWasPressed()) shot.side = ShotCalculator.Side.BACK;
        if (gamepad1.dpadDownWasPressed()) shot.side = ShotCalculator.Side.FRONT;
        if (gamepad1.backWasPressed()) odometry.setPose(START_X, START_Y, Math.toRadians(START_HEADING));

        double speed = gamepad1.left_bumper ? SLOW_SPEED : 1.0;
        double forward = -gamepad1.left_stick_y * speed;
        double strafe = gamepad1.left_stick_x * speed;
        double turn = gamepad1.right_stick_x * speed;
        if (gamepad1.left_trigger > 0.5) {
            turn = Math.max(-MAX_AIM_TURN, Math.min(MAX_AIM_TURN, -shot.getAimError() * AIM_GAIN));
        }
        drive.drive(forward, strafe, turn);

        if (gamepad1.rightBumperWasPressed()) intakeOn = !intakeOn;
        boolean full = transfer.isBallHeld();
        intake.setPower(intakeOn && !full ? INTAKE_POWER : 0);

        if (gamepad1.aWasPressed()) shooterOn = !shooterOn;
        double targetRpm = shooterOn && shot.isReachable() ? shot.getRpm() : 0;
        shooter.setRPM(targetRpm);
        double actualRpm = shooter.getVelocity() * 60 / SHOOTER_TICKS_PER_REV;
        boolean ready = targetRpm > 0 && Math.abs(actualRpm - targetRpm) < RPM_TOLERANCE;

        boolean fire = gamepad1.right_trigger > 0.5 && ready;
        trigger.setPosition(fire ? TRIGGER_FIRE_POSITION : TRIGGER_REST_POSITION);

        telemetry.addData("Hive / side", shot.hive + " / " + shot.side);
        telemetry.addData("Pose", "%.1f, %.1f, %.0f°", odometry.getX(), odometry.getY(), odometry.getHeadingDegrees());
        telemetry.addData("Distance", "%.1f in", shot.getDistance());
        telemetry.addData("Aim error", "%.1f°", shot.getAimError());
        telemetry.addData("Shooter", "%.0f / %.0f rpm %s", actualRpm, targetRpm, ready ? "READY" : "");
        telemetry.addData("Can shoot", shot.canShoot());
        telemetry.addData("Intake", full ? "FULL" : (intakeOn ? "on" : "off"));
    }

    @Override
    public void stop() {
        drive.drive(0, 0, 0);
        intake.setPower(0);
        shooter.setPower(0);
    }
}
