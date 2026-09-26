package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Motor{
    DcMotorEx motor;
    double ticksPerRevolution;
    public Motor(DcMotorEx motor, double ticksPerRevolution){
        this.motor = motor;
        this.ticksPerRevolution = ticksPerRevolution;
    }

    public void setRPM(double rpm){
        motor.setVelocity(ticksPerRevolution * rpm / 60);
    }
}
