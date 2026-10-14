package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;

@Autonomous(name = "Pathfinding")
public class Pathfinding extends OpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(56, 8, 90);
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose  = poseFactory.of(72, 48, 90);

    private enum State { SHOOT_PRELOAD, SEARCH, COLLECT, RETURN, SHOOT, AVOID, PARK, DONE }

    private Follower follower;
    private State state = State.SHOOT_PRELOAD;
    private State stateBeforeAvoid = State.SEARCH;
    private Odometry odometry;
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private Servo triggerServo;

    private boolean busy = false;
    private int ballsHeld = 0;
    private static final int BALLS_TO_COLLECT = 3;
    private static final double PARK_TIME_SECONDS = 25.0;
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double DEFAULT_ROBOT_SIZE_INCHES = 18.0;
    private static final double DEFAULT_MAX_SPEED_INCHES_PER_SECOND = 30.0;
    private static final double DEFAULT_TURN_SPEED_DEGREES_PER_SECOND = 180.0;
    private static final double SIMULATION_STEP_SECONDS = 0.05;
    private static final double OBSTACLE_LOOKAHEAD_INCHES = 18.0;
    private static final double OBSTACLE_MARGIN_INCHES = 2.0;
    private static final String SHOOTER_MOTOR_NAME = "shooter";
    private static final String INTAKE_MOTOR_NAME = "intake";
    private static final String TRIGGER_SERVO_NAME = "trigger";
    private static final double SHOOTER_POWER = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double TRIGGER_REST_POSITION = 0.0;
    private static final double TRIGGER_FIRE_POSITION = 0.45;

    private final SimField field = SimField.bioBuzz();
    private final boolean[] collectedPollen = new boolean[field.pollen.size()];
    private int selectedPollenIndex = -1;

    private Command goTo(Pose from, Pose to) {
        SimPoint waypoint = field.routeAroundHive(toSimPoint(from), toSimPoint(to), DEFAULT_ROBOT_SIZE_INCHES);
        if (waypoint == null) {
            return follow(follower, line(from, to).linear(from, to));
        }

        Pose aroundHive = poseFactory.of(
                waypoint.x,
                waypoint.y,
                headingDegrees(waypoint.x, waypoint.y, to.x(), to.y()));
        return sequential(
                follow(follower, line(from, aroundHive).linear(from, aroundHive)),
                follow(follower, line(aroundHive, to).linear(aroundHive, to))
        );
    }

    public static void main(String[] args) {
        simulate();
    }

    public static void simulate() {
        Locale.setDefault(Locale.US);

        Scanner scanner = new Scanner(System.in);
        System.out.println("BIOBUZZ Pathfinding Simulator");
        System.out.println("Coordinates are in inches on a 144 x 144 FTC field.");
        System.out.println("X increases left-to-right, Y increases front-to-back.");
        System.out.println("Press Enter to use the default value in brackets.");
        System.out.println();

        SimPose start = new SimPose(
                readDouble(scanner, "Start X", 56.0),
                readDouble(scanner, "Start Y", 8.0),
                readDouble(scanner, "Start heading degrees", 90.0)
        );
        SimPoint target = new SimPoint(
                readDouble(scanner, "Final target X", 72.0),
                readDouble(scanner, "Final target Y", 48.0)
        );
        double robotSize = readDouble(scanner, "Robot square size inches", DEFAULT_ROBOT_SIZE_INCHES);
        double maxSpeed = readDouble(scanner, "Max speed inches/second", DEFAULT_MAX_SPEED_INCHES_PER_SECOND);
        double turnSpeed = readDouble(scanner, "Turn speed degrees/second", DEFAULT_TURN_SPEED_DEGREES_PER_SECOND);
        int ballsToCollect = (int) readDouble(scanner, "Balls to collect", BALLS_TO_COLLECT);

        SimField field = SimField.bioBuzz();
        List<SimPoint> path = buildSimulatedPath(start, target, ballsToCollect, field, robotSize);
        SimResult result = runRobotSimulation(start, path, field, robotSize, maxSpeed, turnSpeed);
        result.print();
    }

    private Command shoot() {
        return sequential(
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
                instant(() -> {
                    if (triggerServo != null) {
                        triggerServo.setPosition(TRIGGER_REST_POSITION);
                    }
                    if (intakeMotor != null) {
                        intakeMotor.setPower(0.0);
                    }
                    if (shooterMotor != null) {
                        shooterMotor.setPower(0.0);
                    }
                })
        );
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
        int nearestIndex = -1;
        double nearestDistance = Double.POSITIVE_INFINITY;

        for (int i = 0; i < field.pollen.size(); i++) {
            if (collectedPollen[i]) {
                continue;
            }

            SimPoint pollen = field.pollen.get(i);
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
        SimPoint pollen = field.pollen.get(nearestIndex);
        return poseFactory.of(
                pollen.x,
                pollen.y,
                headingDegrees(here.x(), here.y(), pollen.x, pollen.y));
    }

    private boolean obstacleDetected() {
        if (follower == null) {
            return false;
        }

        Pose here = follower.pose();
        if (isKnownObstacle(here)) {
            return true;
        }

        double heading = here.heading();
        Pose lookahead = poseFactory.of(
                here.x() + OBSTACLE_LOOKAHEAD_INCHES * Math.cos(heading),
                here.y() + OBSTACLE_LOOKAHEAD_INCHES * Math.sin(heading),
                Math.toDegrees(heading));
        return isKnownObstacle(lookahead);
    }

    private boolean isKnownObstacle(Pose here) {
        SimPoint robotCenter = toSimPoint(here);
        double robotRadius = robotRadius(DEFAULT_ROBOT_SIZE_INCHES);
        for (SimObstacle obstacle : field.obstacles) {
            if (obstacle.clearance(robotCenter) <= robotRadius + OBSTACLE_MARGIN_INCHES) {
                return true;
            }
        }
        return false;
    }

    private SimPoint toSimPoint(Pose pose) {
        return new SimPoint(pose.x(), pose.y());
    }

    private static double robotRadius(double robotSize) {
        return Math.sqrt(robotSize * robotSize + robotSize * robotSize) / 2.0;
    }

    private static double headingDegrees(double fromX, double fromY, double toX, double toY) {
        return Math.toDegrees(Math.atan2(toY - fromY, toX - fromX));
    }

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap);
        initGamepieceHardware();
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    private void initGamepieceHardware() {
        shooterMotor = optionalHardware(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        intakeMotor = optionalHardware(DcMotorEx.class, INTAKE_MOTOR_NAME);
        triggerServo = optionalHardware(Servo.class, TRIGGER_SERVO_NAME);

        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (triggerServo != null) {
            triggerServo.setPosition(TRIGGER_REST_POSITION);
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
        follower.update();
        Scheduler.execute();

        if (busy && state != State.AVOID && obstacleDetected()) {
            Scheduler.reset();
            follower.stop();
            stateBeforeAvoid = state;
            state = State.AVOID;
            busy = false;
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
                run(sequential(goTo(here, scorePose), shoot()), State.SEARCH);
                break;

            case SEARCH:
                if (getRuntime() > PARK_TIME_SECONDS) {
                    state = State.PARK;
                    break;
                }
                Pose ball = nearestBall();
                if (ball != null) {
                    run(collectAt(here, ball), State.COLLECT);
                }
                break;

            case COLLECT:
                if (selectedPollenIndex >= 0) {
                    collectedPollen[selectedPollenIndex] = true;
                    selectedPollenIndex = -1;
                }
                ballsHeld++;
                state = (ballsHeld >= BALLS_TO_COLLECT) ? State.RETURN : State.SEARCH;
                break;

            case RETURN:
                run(goTo(here, scorePose), State.SHOOT);
                break;

            case SHOOT:
                run(shoot(), State.SEARCH);
                ballsHeld = 0;
                break;

            case AVOID:
                if (isKnownObstacle(here)) {
                    Pose side = poseFactory.of(here.x() + 12, here.y(), Math.toDegrees(here.heading()));
                    run(goTo(here, side), stateBeforeAvoid);
                } else {
                    double h = here.heading();
                    Pose forward = poseFactory.of(
                            here.x() + 8 * Math.cos(h),
                            here.y() + 8 * Math.sin(h),
                            Math.toDegrees(h));
                    run(goTo(here, forward), stateBeforeAvoid);
                }
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

    private static List<SimPoint> buildSimulatedPath(SimPose start, SimPoint target, int ballsToCollect, SimField field,
                                                     double robotSize) {
        List<SimPoint> path = new ArrayList<>();
        path.add(new SimPoint(start.x, start.y));

        SimPoint scorePoint = new SimPoint(48.0, 48.0);
        addPathPoint(path, field.routeAroundHive(path.get(path.size() - 1), scorePoint, robotSize));
        path.add(scorePoint);

        for (int i = 0; i < ballsToCollect && i < field.pollen.size(); i++) {
            SimPoint pollen = field.pollen.get(i);
            addPathPoint(path, field.routeAroundHive(path.get(path.size() - 1), pollen, robotSize));
            path.add(pollen);
        }

        addPathPoint(path, field.routeAroundHive(path.get(path.size() - 1), scorePoint, robotSize));
        path.add(scorePoint);

        addPathPoint(path, field.routeAroundHive(path.get(path.size() - 1), target, robotSize));
        path.add(target);
        return path;
    }

    private static void addPathPoint(List<SimPoint> path, SimPoint point) {
        if (point == null) {
            return;
        }
        SimPoint previous = path.get(path.size() - 1);
        if (distance(previous, point) > 0.01) {
            path.add(point);
        }
    }

    private static SimResult runRobotSimulation(
            SimPose start,
            List<SimPoint> path,
            SimField field,
            double robotSize,
            double maxSpeed,
            double turnSpeed) {
        SimPose robot = new SimPose(start.x, start.y, start.headingDegrees);
        List<SimPose> samples = new ArrayList<>();
        List<String> collisions = new ArrayList<>();
        double totalDistance = 0.0;
        double totalTime = 0.0;
        double robotRadius = Math.sqrt(robotSize * robotSize + robotSize * robotSize) / 2.0;

        samples.add(robot.copy());
        addCollisions(collisions, field.collisions(robot, robotRadius));

        for (int i = 1; i < path.size(); i++) {
            SimPoint waypoint = path.get(i);
            int guard = 0;
            while (distance(robot, waypoint) > 0.75 && guard < 10000) {
                double wantedHeading = Math.toDegrees(Math.atan2(waypoint.y - robot.y, waypoint.x - robot.x));
                double turnError = angleWrapDegrees(wantedHeading - robot.headingDegrees);
                double turnStep = clamp(turnError,
                        -turnSpeed * SIMULATION_STEP_SECONDS,
                        turnSpeed * SIMULATION_STEP_SECONDS);

                robot.headingDegrees = angleWrapDegrees(robot.headingDegrees + turnStep);
                double alignment = Math.max(0.15, Math.cos(Math.toRadians(turnError)));
                double moveStep = Math.min(maxSpeed * alignment * SIMULATION_STEP_SECONDS, distance(robot, waypoint));

                double oldX = robot.x;
                double oldY = robot.y;
                robot.x += Math.cos(Math.toRadians(robot.headingDegrees)) * moveStep;
                robot.y += Math.sin(Math.toRadians(robot.headingDegrees)) * moveStep;

                totalDistance += distance(oldX, oldY, robot.x, robot.y);
                totalTime += SIMULATION_STEP_SECONDS;
                samples.add(robot.copy());

                addCollisions(collisions, field.collisions(robot, robotRadius));
                guard++;
            }

            if (guard >= 10000) {
                addUnique(collisions, "motion did not settle at waypoint " + i);
                break;
            }
        }

        boolean success = collisions.isEmpty() && distance(robot, path.get(path.size() - 1)) <= 2.0;
        return new SimResult(success, path, samples, totalDistance, totalTime, collisions, field);
    }

    private static double readDouble(Scanner scanner, String prompt, double defaultValue) {
        while (true) {
            System.out.print(prompt + " [" + format(defaultValue) + "]: ");
            String line = scanner.nextLine().trim();
            if (line.isEmpty()) {
                return defaultValue;
            }
            try {
                return Double.parseDouble(line);
            } catch (NumberFormatException e) {
                System.out.println("Please type a number.");
            }
        }
    }

    private static void addCollisions(List<String> collisions, List<String> newCollisions) {
        for (String collision : newCollisions) {
            addUnique(collisions, collision);
        }
    }

    private static void addUnique(List<String> list, String value) {
        if (!list.contains(value)) {
            list.add(value);
        }
    }

    private static double distance(SimPose pose, SimPoint point) {
        return distance(pose.x, pose.y, point.x, point.y);
    }

    private static double distance(SimPoint a, SimPoint b) {
        return distance(a.x, a.y, b.x, b.y);
    }

    private static double distance(double ax, double ay, double bx, double by) {
        return Math.hypot(ax - bx, ay - by);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double angleWrapDegrees(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        } else if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    private static String format(double value) {
        return String.format(Locale.US, "%.2f", value);
    }

    private static class SimResult {
        private final boolean success;
        private final List<SimPoint> path;
        private final List<SimPose> samples;
        private final double distanceInches;
        private final double timeSeconds;
        private final List<String> collisions;
        private final SimField field;

        private SimResult(boolean success, List<SimPoint> path, List<SimPose> samples, double distanceInches,
                          double timeSeconds, List<String> collisions, SimField field) {
            this.success = success;
            this.path = path;
            this.samples = samples;
            this.distanceInches = distanceInches;
            this.timeSeconds = timeSeconds;
            this.collisions = collisions;
            this.field = field;
        }

        private void print() {
            System.out.println();
            System.out.println("Simulation complete");
            System.out.println("Status: " + (success ? "SUCCESS" : "FAILURE"));
            System.out.println();

            System.out.println("Final path:");
            for (int i = 0; i < path.size(); i++) {
                System.out.println("  " + (i + 1) + ". " + path.get(i));
            }

            SimPose last = samples.get(samples.size() - 1);
            System.out.println();
            System.out.println("Final robot pose: " + last);
            System.out.println("Distance: " + format(distanceInches) + " in");
            System.out.println("Estimated time: " + format(timeSeconds) + " sec");
            System.out.println("Motion samples: " + samples.size());
            System.out.println("Collisions: " + (collisions.isEmpty() ? "none" : collisions));
            System.out.println();

            System.out.println("BIOBUZZ field model:");
            for (SimObstacle obstacle : field.obstacles) {
                System.out.println("  - " + obstacle.name + ": " + obstacle);
            }
        }
    }

    private static class SimField {
        private final List<SimObstacle> obstacles = new ArrayList<>();
        private final List<SimPoint> pollen = new ArrayList<>();

        private static SimField bioBuzz() {
            SimField field = new SimField();

            field.obstacles.add(SimObstacle.rectangle("center HIVE frame", 72.0, 72.0, 49.46, 38.95));
            field.obstacles.add(SimObstacle.circle("front-left FLOWER", 24.0, 0.0, 6.0));
            field.obstacles.add(SimObstacle.circle("front-right FLOWER", 120.0, 0.0, 6.0));
            field.obstacles.add(SimObstacle.circle("back-left FLOWER", 24.0, 144.0, 6.0));
            field.obstacles.add(SimObstacle.circle("back-right FLOWER", 120.0, 144.0, 6.0));

            field.pollen.add(new SimPoint(36.0, 24.0));
            field.pollen.add(new SimPoint(108.0, 24.0));
            field.pollen.add(new SimPoint(36.0, 120.0));
            field.pollen.add(new SimPoint(108.0, 120.0));
            field.pollen.add(new SimPoint(72.0, 24.0));
            field.pollen.add(new SimPoint(72.0, 120.0));

            return field;
        }

        private SimPoint routeAroundHive(SimPoint from, SimPoint to, double robotSize) {
            SimObstacle hive = obstacles.get(0);
            double robotRadius = Math.sqrt(robotSize * robotSize + robotSize * robotSize) / 2.0;
            if (!hive.segmentIntersects(from, to, robotRadius)) {
                return null;
            }

            double aboveY = hive.centerY + hive.height / 2.0 + robotRadius + 2.0;
            double belowY = hive.centerY - hive.height / 2.0 - robotRadius - 2.0;
            SimPoint above = new SimPoint(72.0, clamp(aboveY, 0.0, FIELD_SIZE_INCHES));
            SimPoint below = new SimPoint(72.0, clamp(belowY, 0.0, FIELD_SIZE_INCHES));
            return distance(from, above) + distance(above, to) < distance(from, below) + distance(below, to)
                    ? above
                    : below;
        }

        private List<String> collisions(SimPose robot, double robotRadius) {
            List<String> collisions = new ArrayList<>();
            if (robot.x - robotRadius < 0.0 || robot.x + robotRadius > FIELD_SIZE_INCHES
                    || robot.y - robotRadius < 0.0 || robot.y + robotRadius > FIELD_SIZE_INCHES) {
                collisions.add("field wall");
            }

            SimPoint center = new SimPoint(robot.x, robot.y);
            for (SimObstacle obstacle : obstacles) {
                if (obstacle.clearance(center) <= robotRadius) {
                    collisions.add(obstacle.name);
                }
            }
            return collisions;
        }
    }

    private static class SimObstacle {
        private final String name;
        private final boolean circle;
        private final double centerX;
        private final double centerY;
        private final double width;
        private final double height;
        private final double radius;

        private SimObstacle(String name, boolean circle, double centerX, double centerY,
                            double width, double height, double radius) {
            this.name = name;
            this.circle = circle;
            this.centerX = centerX;
            this.centerY = centerY;
            this.width = width;
            this.height = height;
            this.radius = radius;
        }

        private static SimObstacle rectangle(String name, double centerX, double centerY, double width, double height) {
            return new SimObstacle(name, false, centerX, centerY, width, height, 0.0);
        }

        private static SimObstacle circle(String name, double centerX, double centerY, double radius) {
            return new SimObstacle(name, true, centerX, centerY, 0.0, 0.0, radius);
        }

        private double clearance(SimPoint point) {
            if (circle) {
                return Math.max(0.0, distance(point.x, point.y, centerX, centerY) - radius);
            }

            double dx = Math.max(Math.abs(point.x - centerX) - width / 2.0, 0.0);
            double dy = Math.max(Math.abs(point.y - centerY) - height / 2.0, 0.0);
            return Math.hypot(dx, dy);
        }

        private boolean segmentIntersects(SimPoint from, SimPoint to, double padding) {
            int checks = 40;
            for (int i = 0; i <= checks; i++) {
                double t = i / (double) checks;
                SimPoint point = new SimPoint(
                        from.x + (to.x - from.x) * t,
                        from.y + (to.y - from.y) * t
                );
                if (clearance(point) <= padding) {
                    return true;
                }
            }
            return false;
        }

        @Override
        public String toString() {
            if (circle) {
                return "circle center=(" + format(centerX) + ", " + format(centerY) + "), radius="
                        + format(radius) + " in";
            }
            return "rectangle center=(" + format(centerX) + ", " + format(centerY) + "), size="
                    + format(width) + " x " + format(height) + " in";
        }
    }

    private static class SimPoint {
        private final double x;
        private final double y;

        private SimPoint(double x, double y) {
            this.x = x;
            this.y = y;
        }

        @Override
        public String toString() {
            return "(" + format(x) + ", " + format(y) + ")";
        }
    }

    private static class SimPose {
        private double x;
        private double y;
        private double headingDegrees;

        private SimPose(double x, double y, double headingDegrees) {
            this.x = x;
            this.y = y;
            this.headingDegrees = headingDegrees;
        }

        private SimPose copy() {
            return new SimPose(x, y, headingDegrees);
        }

        @Override
        public String toString() {
            return "(" + format(x) + ", " + format(y) + "), heading " + format(headingDegrees) + " deg";
        }
    }
}
/*Pseudocode for pathfinding ig
game starts
if we shoot first, shoot le balls
if not, wait for hive to tip, update the current tip of the hive, start collecting balls
Call move functions until sensor detects ball
Robot receives coordinates of the ball
Robot calls move function to get to the ball
Robot also activates the intake
Robot repeats until it has enough balls (3-4)
Robot then moves into position to fire
Robot fires
U can use apriltags to see if the hive tipped or not
Update most recent hive tip and which hive is down
----------IMPORTANT-----------
If at any point a sensor detects a obstacle
See if it is at a preregistered obstacle point
if is, go around it
if not, it is a robot
Continue going forward at a fast velocity (hopefully butts then out of the way)
If still there, go sideways, then forwards, then continue
----------OTHER----------
If we had the most recent tip
Still collect balls
Once we have collected the balls
position urself in front of the tipped hive
Send the ball beneath the hive (hopefully) giving it to ur teammate
===========================
If robot detects enemy nectar (color sensor)
reverses intake
hits it out of the way ig
*/
