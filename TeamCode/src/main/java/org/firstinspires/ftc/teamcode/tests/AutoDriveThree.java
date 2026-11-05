package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Drive;

//
//This one is if our robot starts in the position IN FRONT OF THE HIVE
//
@Autonomous
public class AutoDriveThree extends OpMode {
    Drive drive;
    ElapsedTime timer = new ElapsedTime();

    int step = 0;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (step == 0) {
            drive.drive(0.3, 0, 90);

            if (timer.seconds() >= 2) {
                step++;
                timer.reset();
            }
        }
        if (step == 1) {
            drive.drive(0.3, 0, 90);

            if (timer.seconds() >= 1.9) {
                step++;
                timer.reset();
            }
        }
        else {
            drive.drive(0, 0, 0);
        }
    }
}