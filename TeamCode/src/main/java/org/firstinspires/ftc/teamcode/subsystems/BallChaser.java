package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class BallChaser {
    public static double TURN_GAIN = 0.02;          // turn power per degree the ball is off-center
    public static double MAX_TURN = 0.5;
    public static double DRIVE_SPEED = 0.4;
    public static double SLOW_ANGLE = 20;           // degrees off-center at which it stops driving and only turns
    public static double KEEP_GOING_SECONDS = 0.5;  // keeps driving this long after the ball drops out of view under the camera
    public static double MAX_STALE_MS = 200;        // ignore Limelight results older than this
    public static String BALL_CLASS = "";           // only chase this detector class, "" = anything detected

    Limelight limelight;
    boolean ballSeen;
    boolean chasing;
    double ballAngle;
    double lastSeenTime;
    double forward;
    double turn;

    public BallChaser(Limelight limelight){
        this.limelight = limelight;
    }

    public void update(){
        LLResult result = limelight.getResult();
        LLResultTypes.DetectorResult nearest = null;
        if (result != null && result.getStaleness() < MAX_STALE_MS){
            for (LLResultTypes.DetectorResult ball : limelight.getDetected()){
                if (!BALL_CLASS.isEmpty() && !BALL_CLASS.equals(ball.getClassName())){
                    continue;
                }
                // The ball that looks biggest is the closest one.
                if (nearest == null || ball.getTargetArea() > nearest.getTargetArea()){
                    nearest = ball;
                }
            }
        }
        update(nearest != null, nearest != null ? nearest.getTargetXDegrees() : 0, System.nanoTime() / 1e9);
    }

    void update(boolean seen, double angle, double time){
        ballSeen = seen;
        if (seen){
            ballAngle = angle;
            lastSeenTime = time;
            chasing = true;
            turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, TURN_GAIN * angle));
            forward = DRIVE_SPEED * Math.max(0, 1 - Math.abs(angle) / SLOW_ANGLE);
        } else if (chasing && time - lastSeenTime < KEEP_GOING_SECONDS){
            turn = 0;
            forward = DRIVE_SPEED;
        } else {
            chasing = false;
            turn = 0;
            forward = 0;
        }
    }

    public double getForward(){
        return forward;
    }
    public double getTurn(){
        return turn;
    }
    public boolean isBallSeen(){
        return ballSeen;
    }
    public boolean isChasing(){
        return chasing;
    }
    public double getBallAngle(){
        return ballAngle;
    }
}
