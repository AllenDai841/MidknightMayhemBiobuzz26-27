package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TransferSensor {
    public double detectDistanceCm = 5;
    public double clearDistanceCm = 8;
    public double holdSeconds = 0.3;

    DistanceSensor sensor;
    double distance;
    boolean ballSeen;
    double seenSince;
    double lastTime;

    public TransferSensor(DistanceSensor sensor){
        this.sensor = sensor;
    }

    public void update(){
        update(sensor.getDistance(DistanceUnit.CM), System.nanoTime() / 1e9);
    }

    void update(double distance, double time){
        this.distance = distance;
        boolean wasSeen = ballSeen;
        ballSeen = distance < (wasSeen ? clearDistanceCm : detectDistanceCm);
        if (ballSeen && !wasSeen){
            seenSince = time;
        }
        lastTime = time;
    }

    public boolean isBallSeen(){
        return ballSeen;
    }
    public boolean isBallHeld(){
        return ballSeen && lastTime - seenSince >= holdSeconds;
    }
    public double getDistance(){
        return distance;
    }
}
