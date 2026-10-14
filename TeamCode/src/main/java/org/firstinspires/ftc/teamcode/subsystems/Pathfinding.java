package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Pathfinding")
public class Pathfinding extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose scorePose = poseFactory.of(48, 36, 90);
    private final Pose parkPose  = poseFactory.of(72, 36, 90);

    private enum State { SHOOT_PRELOAD, SEARCH, COLLECT, RETURN, SHOOT, AVOID, PARK, DONE }

    private Follower follower;
    private State state = State.SHOOT_PRELOAD;
    private State stateBeforeAvoid = State.SEARCH;
    private Odometry odometry;
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Servo triggerServo;
    private Limelight limelight;
    private BallChaser ballChaser;
    private AprilTagWebcam arducam;
    private HiveAimer hiveAimer;
    private TransferSensor transferSensor;

    private boolean busy = false;
    private boolean driving = false;
    private Pose goal = startPose;
    private double avoidCooldownUntil = 0.0;
    private int ballsHeld = 0;
    private static final int BALLS_TO_COLLECT = 3;
    private static final double PARK_TIME_SECONDS = 25.0;
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double DEFAULT_ROBOT_SIZE_INCHES = 18.0;
    private static final double OBSTACLE_LOOKAHEAD_INCHES = 18.0;
    private static final double OBSTACLE_MARGIN_INCHES = 2.0;
    private static final double AVOID_STEP_INCHES = 12.0;
    private static final double AVOID_COOLDOWN_SECONDS = 1.0;
    private static final String SHOOTER_MOTOR_NAME = "shooter";
    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final String TRIGGER_SERVO_NAME = "trigger";
    private static final String LIMELIGHT_NAME = "limelight";
    private static final String APRILTAG_CAMERA_NAME = "arducam";
    private static final String TRANSFER_SENSOR_NAME = "transfer";
    private static final double SHOOTER_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRIGGER_REST_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.45;
    private static final double VISION_BALL_TARGET_DISTANCE_INCHES = 24.0;

    private final FieldModel field = FieldModel.bioBuzz();
    private final boolean[] collectedPollen = new boolean[field.pollen.size()];
    private int selectedPollenIndex = -1;

    private Command leg(Pose from, Pose to) {
        return sequential(
                instant(() -> {
                    goal = to;
                    driving = true;
                }),
                follow(follower, line(from, to).linear(from, to))
        );
    }

    private Command goTo(Pose from, Pose to) {
        FieldPoint waypoint = field.routeAroundHive(toFieldPoint(from), toFieldPoint(to), DEFAULT_ROBOT_SIZE_INCHES);
        Command path;
        if (waypoint == null) {
            path = leg(from, to);
        } else {
            Pose aroundHive = poseFactory.of(
                    waypoint.x,
                    waypoint.y,
                    headingDegrees(waypoint.x, waypoint.y, to.x(), to.y()));
            path = sequential(leg(from, aroundHive), leg(aroundHive, to));
        }
        return sequential(path, instant(() -> driving = false));
    }

    private Command shoot() {
        return sequential(
                instant(() -> {
                    driving = false;
                    if (hiveAimer != null) {
                        hiveAimer.update();
                    }
                }),
                instant(() -> {
                    if (shooterMotor != null) {
                        shooterMotor.setPower(SHOOTER_POWER);
                    }
                }),
                waitMs(800),
                instant(() -> {
                    if (triggerServo != null) {
                        triggerServo.setPosition(TRIGGER_FIRE_POSITION);
                    } else if (intakeMotor != null) {
                        intakeMotor.setPower(INTAKE_POWER);
                    }
                }),
                waitMs(1000),
                instant(this::stopMechanisms)
        );
    }

    private void stopMechanisms() {
        if (triggerServo != null) {
            triggerServo.setPosition(TRIGGER_REST_POSITION);
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
    }

    private Command intakeOn() {
        return instant(() -> {
            if (intakeMotor != null) {
                intakeMotor.setPower(INTAKE_POWER);
            }
        });
    }

    private Command intakeOff() {
        return instant(() -> {
            if (intakeMotor != null) {
                intakeMotor.setPower(0.0);
            }
        });
    }

    private Command collectAt(Pose from, Pose ball) {
        return sequential(
                intakeOn(),
                goTo(from, ball),
                waitMs(300),
                intakeOff()
        );
    }

    private Pose nearestBall() {
        Pose here = follower == null ? startPose : follower.pose();
        if (transferSensor != null && transferSensor.isBallSeen()) {
            selectedPollenIndex = -1;
            return null;
        }

        Pose visionBall = nearestVisionBall(here);
        if (visionBall != null) {
            selectedPollenIndex = -1;
            return visionBall;
        }

        int nearestIndex = -1;
        double nearestDistance = Double.POSITIVE_INFINITY;

        for (int i = 0; i < field.pollen.size(); i++) {
            if (collectedPollen[i]) {
                continue;
            }

            FieldPoint pollen = field.pollen.get(i);
            double distance = distance(here.x(), here.y(), pollen.x, pollen.y);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestIndex = i;
            }
        }

        if (nearestIndex < 0) {
            selectedPollenIndex = -1;
            return null;
        }

        selectedPollenIndex = nearestIndex;
        FieldPoint pollen = field.pollen.get(nearestIndex);
        return poseFactory.of(
                pollen.x,
                pollen.y,
                headingDegrees(here.x(), here.y(), pollen.x, pollen.y));
    }

    private Pose nearestVisionBall(Pose here) {
        if (ballChaser == null) {
            return null;
        }

        ballChaser.update();
        if (!ballChaser.isChasing()) {
            return null;
        }

        double targetHeading = here.heading() - Math.toRadians(ballChaser.getBallAngle());
        double x = clamp(
                here.x() + VISION_BALL_TARGET_DISTANCE_INCHES * Math.cos(targetHeading),
                DEFAULT_ROBOT_SIZE_INCHES / 2.0,
                FIELD_SIZE_INCHES - DEFAULT_ROBOT_SIZE_INCHES / 2.0);
        double y = clamp(
                here.y() + VISION_BALL_TARGET_DISTANCE_INCHES * Math.sin(targetHeading),
                DEFAULT_ROBOT_SIZE_INCHES / 2.0,
                FIELD_SIZE_INCHES - DEFAULT_ROBOT_SIZE_INCHES / 2.0);
        return poseFactory.of(x, y, Math.toDegrees(targetHeading));
    }

    private boolean obstacleDetected() {
        if (follower == null) {
            return false;
        }

        Pose here = follower.pose();
        if (isKnownObstacle(here.x(), here.y())) {
            return true;
        }

        double remaining = distance(here.x(), here.y(), goal.x(), goal.y());
        if (remaining < 1.0) {
            return false;
        }

        double reach = Math.min(OBSTACLE_LOOKAHEAD_INCHES, remaining);
        double lookX = here.x() + (goal.x() - here.x()) / remaining * reach;
        double lookY = here.y() + (goal.y() - here.y()) / remaining * reach;
        return isKnownObstacle(lookX, lookY);
    }

    private boolean isKnownObstacle(double x, double y) {
        FieldPoint robotCenter = new FieldPoint(x, y);
        double radius = robotRadius(DEFAULT_ROBOT_SIZE_INCHES);
        for (FieldObstacle obstacle : field.obstacles) {
            if (obstacle.clearance(robotCenter) <= radius + OBSTACLE_MARGIN_INCHES) {
                return true;
            }
        }
        return false;
    }

    private Pose awayFromObstacle(Pose here) {
        FieldPoint center = toFieldPoint(here);
        FieldObstacle nearest = null;
        double best = Double.POSITIVE_INFINITY;
        for (FieldObstacle obstacle : field.obstacles) {
            double clearance = obstacle.clearance(center);
            if (clearance < best) {
                best = clearance;
                nearest = obstacle;
            }
        }

        double dx = 0.0;
        double dy = -1.0;
        if (nearest != null) {
            double ox = here.x() - nearest.centerX;
            double oy = here.y() - nearest.centerY;
            double length = Math.hypot(ox, oy);
            if (length > 1e-6) {
                dx = ox / length;
                dy = oy / length;
            }
        }

        double edge = DEFAULT_ROBOT_SIZE_INCHES / 2.0;
        double x = clamp(here.x() + AVOID_STEP_INCHES * dx, edge, FIELD_SIZE_INCHES - edge);
        double y = clamp(here.y() + AVOID_STEP_INCHES * dy, edge, FIELD_SIZE_INCHES - edge);
        return poseFactory.of(x, y, Math.toDegrees(here.heading()));
    }

    private FieldPoint toFieldPoint(Pose pose) {
        return new FieldPoint(pose.x(), pose.y());
    }

    private static double robotRadius(double robotSize) {
        return Math.sqrt(robotSize * robotSize + robotSize * robotSize) / 2.0;
    }

    private static double headingDegrees(double fromX, double fromY, double toX, double toY) {
        return Math.toDegrees(Math.atan2(toY - fromY, toX - fromX));
    }

    private Pose aimedScorePose() {
        return poseFactory.of(
                scorePose.x(),
                scorePose.y(),
                headingDegrees(scorePose.x(), scorePose.y(), HiveAimer.HIVE_X, HiveAimer.HIVE_Y));
    }

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap);
        initGamepieceHardware();
        initVisionHardware();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    private void initGamepieceHardware() {
        shooterMotor = optionalHardware(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        intakeMotor = optionalHardware(DcMotorEx.class, INTAKE_MOTOR_NAME);
        triggerServo = optionalHardware(Servo.class, TRIGGER_SERVO_NAME);
        stopMechanisms();
    }

    private void initVisionHardware() {
        Limelight3A limelightDevice = optionalHardware(Limelight3A.class, LIMELIGHT_NAME);
        if (limelightDevice != null) {
            limelight = new Limelight(limelightDevice);
            limelight.start();
            ballChaser = new BallChaser(limelight);
        }

        DistanceSensor transferDistanceSensor = optionalHardware(DistanceSensor.class, TRANSFER_SENSOR_NAME);
        if (transferDistanceSensor != null) {
            transferSensor = new TransferSensor(transferDistanceSensor);
        }

        arducam = new AprilTagWebcam();
        try {
            arducam.init(hardwareMap, telemetry, APRILTAG_CAMERA_NAME);
            hiveAimer = new HiveAimer(arducam, odometry);
        } catch (IllegalArgumentException e) {
            arducam = null;
            telemetry.addData("Missing optional hardware", APRILTAG_CAMERA_NAME);
        }
    }

    private <T> T optionalHardware(Class<? extends T> hardwareClass, String name) {
        try {
            return hardwareMap.get(hardwareClass, name);
        } catch (IllegalArgumentException e) {
            telemetry.addData("Missing optional hardware", name);
            return null;
        }
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        odometry.update();
        if (transferSensor != null) {
            transferSensor.update();
        }
        if (ballChaser != null) {
            ballChaser.update();
        }
        if (hiveAimer != null) {
            hiveAimer.update();
        }

        follower.update();
        Scheduler.execute();

        if (busy
                && driving
                && state != State.AVOID
                && getRuntime() > avoidCooldownUntil
                && obstacleDetected()) {
            stopMechanisms();
            Scheduler.reset();
            follower.stop();
            stateBeforeAvoid = state;
            state = State.AVOID;
            busy = false;
            driving = false;
        }

        if (!busy) advance();

        telemetry.addData("State", state);
        telemetry.addData("Busy", busy);
        telemetry.addData("Balls", ballsHeld);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    private void advance() {
        Pose here = follower.pose();

        switch (state) {
            case SHOOT_PRELOAD:
                run(sequential(goTo(here, aimedScorePose()), shoot()), State.SEARCH);
                break;

            case SEARCH:
                if (getRuntime() > PARK_TIME_SECONDS) {
                    state = State.PARK;
                    break;
                }
                Pose ball = nearestBall();
                if (ball != null) {
                    run(collectAt(here, ball), State.COLLECT);
                } else if (ballsHeld > 0 || (transferSensor != null && transferSensor.isBallSeen())) {
                    state = State.RETURN;
                }
                break;

            case COLLECT:
                if (transferSensor != null && !transferSensor.isBallHeld()) {
                    state = State.SEARCH;
                    break;
                }
                if (selectedPollenIndex >= 0) {
                    collectedPollen[selectedPollenIndex] = true;
                    selectedPollenIndex = -1;
                }
                ballsHeld++;
                state = (ballsHeld >= BALLS_TO_COLLECT) ? State.RETURN : State.SEARCH;
                break;

            case RETURN:
                run(goTo(here, aimedScorePose()), State.SHOOT);
                break;

            case SHOOT:
                run(shoot(), State.SEARCH);
                ballsHeld = 0;
                break;

            case AVOID:
                Pose away = awayFromObstacle(here);
                run(sequential(
                        leg(here, away),
                        instant(() -> {
                            driving = false;
                            avoidCooldownUntil = getRuntime() + AVOID_COOLDOWN_SECONDS;
                        })
                ), stateBeforeAvoid);
                break;

            case PARK:
                run(goTo(here, parkPose), State.DONE);
                break;

            case DONE:
                break;
        }
    }

    private void run(Command c, State next) {
        busy = true;
        schedule(sequential(c, instant(() -> {
            state = next;
            busy = false;
        })));
    }

    @Override
    public void stop() {
        stopMechanisms();
        if (limelight != null) {
            limelight.stop();
        }
        if (arducam != null) {
            arducam.stop();
        }
    }

    private static double distance(FieldPoint a, FieldPoint b) {
        return distance(a.x, a.y, b.x, b.y);
    }

    private static double distance(double ax, double ay, double bx, double by) {
        return Math.hypot(ax - bx, ay - by);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static class FieldModel {
        private final List<FieldObstacle> obstacles = new ArrayList<>();
        private final List<FieldPoint> pollen = new ArrayList<>();

        private static FieldModel bioBuzz() {
            FieldModel field = new FieldModel();

            field.obstacles.add(FieldObstacle.rectangle(72.0, 72.0, 49.46, 38.95));
            field.obstacles.add(FieldObstacle.circle(24.0, 0.0, 6.0));
            field.obstacles.add(FieldObstacle.circle(120.0, 0.0, 6.0));
            field.obstacles.add(FieldObstacle.circle(24.0, 144.0, 6.0));
            field.obstacles.add(FieldObstacle.circle(120.0, 144.0, 6.0));

            field.pollen.add(new FieldPoint(36.0, 24.0));
            field.pollen.add(new FieldPoint(108.0, 24.0));
            field.pollen.add(new FieldPoint(36.0, 120.0));
            field.pollen.add(new FieldPoint(108.0, 120.0));
            field.pollen.add(new FieldPoint(72.0, 24.0));
            field.pollen.add(new FieldPoint(72.0, 120.0));

            return field;
        }

        private FieldPoint routeAroundHive(FieldPoint from, FieldPoint to, double robotSize) {
            FieldObstacle hive = obstacles.get(0);
            double robotRadius = Math.sqrt(robotSize * robotSize + robotSize * robotSize) / 2.0;
            if (!hive.segmentIntersects(from, to, robotRadius)) {
                return null;
            }

            double detourClearance = robotRadius + OBSTACLE_MARGIN_INCHES + 2.0;
            double aboveY = hive.centerY + hive.height / 2.0 + detourClearance;
            double belowY = hive.centerY - hive.height / 2.0 - detourClearance;
            FieldPoint above = new FieldPoint(72.0, clamp(aboveY, 0.0, FIELD_SIZE_INCHES));
            FieldPoint below = new FieldPoint(72.0, clamp(belowY, 0.0, FIELD_SIZE_INCHES));
            return distance(from, above) + distance(above, to) < distance(from, below) + distance(below, to)
                    ? above
                    : below;
        }
    }

    private static class FieldObstacle {
        private final boolean circle;
        private final double centerX;
        private final double centerY;
        private final double width;
        private final double height;
        private final double radius;

        private FieldObstacle(boolean circle, double centerX, double centerY, double width, double height,
                              double radius) {
            this.circle = circle;
            this.centerX = centerX;
            this.centerY = centerY;
            this.width = width;
            this.height = height;
            this.radius = radius;
        }

        private static FieldObstacle rectangle(double centerX, double centerY, double width, double height) {
            return new FieldObstacle(false, centerX, centerY, width, height, 0.0);
        }

        private static FieldObstacle circle(double centerX, double centerY, double radius) {
            return new FieldObstacle(true, centerX, centerY, 0.0, 0.0, radius);
        }

        private double clearance(FieldPoint point) {
            if (circle) {
                return Math.max(0.0, distance(point.x, point.y, centerX, centerY) - radius);
            }

            double dx = Math.max(Math.abs(point.x - centerX) - width / 2.0, 0.0);
            double dy = Math.max(Math.abs(point.y - centerY) - height / 2.0, 0.0);
            return Math.hypot(dx, dy);
        }

        private boolean segmentIntersects(FieldPoint from, FieldPoint to, double padding) {
            int checks = 40;
            for (int i = 0; i <= checks; i++) {
                double t = i / (double) checks;
                FieldPoint point = new FieldPoint(
                        from.x + (to.x - from.x) * t,
                        from.y + (to.y - from.y) * t
                );
                if (clearance(point) <= padding) {
                    return true;
                }
            }
            return false;
        }
    }

    private static class FieldPoint {
        private final double x;
        private final double y;

        private FieldPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}