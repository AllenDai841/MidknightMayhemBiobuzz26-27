package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous")
public class Autonomous extends OpMode {

    private static final double FIELD_SIZE_INCHES = 144.0;

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(8.663813215472306, 38.54073569389506, 0);
    private final Pose scorePose = poseFactory.of(59.707276792047374, 130.28426891272883, 90);
    private final Pose parkPose  = poseFactory.of(8.663813215472306, 38.54073569389506, 0);

    private enum State { DRIVE_TO_SCORE, PARK, AVOID, DONE }

    private Follower follower;
    private State state = State.DRIVE_TO_SCORE;
    private State stateBeforeAvoid = State.DRIVE_TO_SCORE;
    private Odometry odometry;

    private boolean busy = false;
    private boolean driving = false;
    private Pose goal = startPose;
    private double avoidCooldownUntil = 0.0;

    private static final double ROBOT_WIDTH_INCHES = 13.5;
    private static final double ROBOT_LENGTH_INCHES = 17.0;
    private static final double OBSTACLE_LOOKAHEAD_INCHES = 18.0;
    private static final double OBSTACLE_MARGIN_INCHES = 2.0;
    private static final double AVOID_STEP_INCHES = 12.0;
    private static final double AVOID_COOLDOWN_SECONDS = 1.0;

    private final FieldModel field = FieldModel.bioBuzz();

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
        FieldPoint waypoint = field.routeAroundHive(toFieldPoint(from), toFieldPoint(to), robotClearanceRadius());
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
        double radius = robotClearanceRadius();
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

        double xEdge = ROBOT_WIDTH_INCHES / 2.0;
        double yEdge = ROBOT_LENGTH_INCHES / 2.0;
        double x = clamp(here.x() + AVOID_STEP_INCHES * dx, xEdge, FIELD_SIZE_INCHES - xEdge);
        double y = clamp(here.y() + AVOID_STEP_INCHES * dy, yEdge, FIELD_SIZE_INCHES - yEdge);
        return poseFactory.of(x, y, Math.toDegrees(here.heading()));
    }

    private FieldPoint toFieldPoint(Pose pose) {
        return new FieldPoint(pose.x(), pose.y());
    }

    private static double robotClearanceRadius() {
        return Math.hypot(ROBOT_WIDTH_INCHES, ROBOT_LENGTH_INCHES) / 2.0;
    }

    private static double headingDegrees(double fromX, double fromY, double toX, double toY) {
        return Math.toDegrees(Math.atan2(toY - fromY, toX - fromX));
    }

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        odometry.setPose(startPose.x(), startPose.y(), startPose.heading());
        follower.update();
    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        odometry.update();
        follower.update();
        Scheduler.execute();

        if (busy
                && driving
                && state != State.AVOID
                && getRuntime() > avoidCooldownUntil
                && obstacleDetected()) {
            Scheduler.reset();
            follower.stop();
            stateBeforeAvoid = state;
            state = State.AVOID;
            busy = false;
            driving = false;
        }

        if (!busy) {
            advance();
        }

        telemetry.addData("State", state);
        telemetry.addData("Busy", busy);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    private void advance() {
        Pose here = follower.pose();

        switch (state) {
            case DRIVE_TO_SCORE:
                run(goTo(here, scorePose), State.PARK);
                break;

            case PARK:
                run(goTo(here, parkPose), State.DONE);
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
        Scheduler.reset();
        if (follower != null) {
            follower.stop();
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

        private static FieldModel bioBuzz() {
            FieldModel field = new FieldModel();

            field.obstacles.add(FieldObstacle.rectangle(72.0, 72.0, 49.46, 38.95));
            field.obstacles.add(FieldObstacle.circle(24.0, 0.0, 6.0));
            field.obstacles.add(FieldObstacle.circle(120.0, 0.0, 6.0));
            field.obstacles.add(FieldObstacle.circle(24.0, 144.0, 6.0));
            field.obstacles.add(FieldObstacle.circle(120.0, 144.0, 6.0));

            return field;
        }

        private FieldPoint routeAroundHive(FieldPoint from, FieldPoint to, double robotRadius) {
            FieldObstacle hive = obstacles.get(0);
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
