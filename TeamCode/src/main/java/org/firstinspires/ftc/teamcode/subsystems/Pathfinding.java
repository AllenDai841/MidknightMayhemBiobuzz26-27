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

import org.firstinspires.ftc.teamcode.pedro.Constants;

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

    private boolean busy = false;
    private int ballsHeld = 0;
    private static final int BALLS_TO_COLLECT = 3;
    private static final double PARK_TIME_SECONDS = 25.0;

    private Command goTo(Pose from, Pose to) {
        return follow(follower, line(from, to).linear(from, to));
    }

    private Command shoot() {
        return sequential(
                instant(() -> { }),
                waitMs(800),
                instant(() -> { }),
                waitMs(1000),
                instant(() -> { })
        );
    }

    private Command intakeOn()  { return instant(() -> { }); }
    private Command intakeOff() { return instant(() -> { }); }

    private Command collectAt(Pose from, Pose ball) {
        return sequential(
                intakeOn(),
                goTo(from, ball),
                waitMs(300),
                intakeOff()
        );
    }

    private Pose nearestBall() {
        return null;
    }

    private boolean obstacleDetected() {
        return false;
    }

    private boolean isKnownObstacle(Pose here) {
        return false;
    }

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap);
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
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