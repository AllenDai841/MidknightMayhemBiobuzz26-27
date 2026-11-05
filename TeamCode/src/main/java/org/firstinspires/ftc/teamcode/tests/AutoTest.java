package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous
public class AutoTest extends OpMode {
    Drive drive;
    ElapsedTime time;
    boolean bool = false;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        time = new ElapsedTime();
    }

    @Override
    public void loop() {
        if(!bool){
            time.reset();
            bool = true;
        }
        while(time.seconds() <= 3) {
            drive.drive(0.5, 0, 0);
        }
    }
}
