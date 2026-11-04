package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

@Autonomous
public class AutoDriveTest extends OpMode {
    Drive drive;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
    }

    @Override
    public void loop() {
        drive.drive(40, 0, 90);
        drive.drive(20,0,90);
        drive.drive(-5,0,0);
    }
}