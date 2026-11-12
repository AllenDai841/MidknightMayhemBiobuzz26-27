package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drive;


@Autonomous(name = "Auto Mid", group = "1 - Match")
public class AutoMid extends OpMode {
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
        if(time.milliseconds() <= 350) {
            drive.drive(0, 0.3, 0);
        }
        else if(time.milliseconds() <= 500){
            drive.stop();
        }
        else if(time.milliseconds() <= 1150) {
            drive.drive(0.3, 0, 0);
        }
        else{
            drive.stop();
        }

    }
}
