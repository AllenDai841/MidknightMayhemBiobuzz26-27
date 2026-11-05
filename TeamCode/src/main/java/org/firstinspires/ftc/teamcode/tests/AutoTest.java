package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous
public class AutoTest extends OpMode {
    Drive drive;
    ElapsedTime time = new ElapsedTime();
    int s = 0;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
    }
    @Override
    public void start(){
        time.reset();
    }

    @Override
    public void loop() {
        if(time.seconds() <= 1) {
            drive.drive(0.5, 0, 0);
        }
        else{
            drive.stop();
        }

    }
}
