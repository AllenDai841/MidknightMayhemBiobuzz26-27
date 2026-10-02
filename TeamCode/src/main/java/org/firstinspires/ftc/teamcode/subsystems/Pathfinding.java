package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

@Autonomous
public class Pathfinding extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose = poseFactory.of(72, 48, 90);

    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }
    private Path park(){
        return line(scorePose, parkPose).linear(scorePose, parkPose);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void start() {
        schedule(follow(follower, park()));
        schedule(autoRoutine());

    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();


        TelemetryMessage telemetryData;
        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
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