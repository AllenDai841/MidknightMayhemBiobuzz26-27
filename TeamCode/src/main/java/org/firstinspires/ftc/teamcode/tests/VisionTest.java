package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.BallChaser;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.HiveAimer;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

@TeleOp
public class VisionTest extends OpMode {
    Drive drive;
    Odometry odometry;
    Limelight limelight;
    AprilTagWebcam arducam;
    BallChaser ballChaser;
    HiveAimer hiveAimer;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        odometry = new Odometry(hardwareMap);
        limelight = new Limelight(hardwareMap.get(Limelight3A.class, "limelight"));
        limelight.start();
        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, telemetry, "arducam");
        ballChaser = new BallChaser(limelight);
        hiveAimer = new HiveAimer(arducam, odometry);
    }

    @Override
    public void loop() {
        odometry.update();
        ballChaser.update();
        hiveAimer.update();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        if (gamepad1.a) {
            // Hold A: drive to the nearest ball.
            forward = ballChaser.getForward();
            strafe = 0;
            turn = ballChaser.getTurn();
        } else if (gamepad1.b) {
            // Hold B: turn toward the hive, still driving with the left stick.
            turn = hiveAimer.getTurn();
        }
        drive.drive(forward, strafe, turn);

        telemetry.addLine("Hold A = chase ball, hold B = aim at hive");
        telemetry.addData("Ball", "seen: %b, angle: %.1f deg, chasing: %b",
                ballChaser.isBallSeen(), ballChaser.getBallAngle(), ballChaser.isChasing());
        telemetry.addData("Hive", "tag seen: %b, error: %.1f deg, aimed: %b",
                hiveAimer.isTagSeen(), hiveAimer.getError(), hiveAimer.isAimed());
        telemetry.addData("Shot", "rpm: %.0f, distance: %.1f in, can shoot: %b",
                hiveAimer.getShotRpm(), hiveAimer.getShotDistance(), hiveAimer.canShoot());
        telemetry.addData("Pose", "%.1f, %.1f, %.1f deg", odometry.getX(), odometry.getY(), odometry.getHeadingDegrees());
    }

    @Override
    public void stop() {
        arducam.stop();
    }
}
