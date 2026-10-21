package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TeleOpOuttakePseudo extends OpMode {
    @Override
    public void init() {
        /*
        get motor and sensor
         */
    }

    @Override
    public void loop() {
        /*
        when robot stops & takes position to shoot, return position odometry (x1,y1)
        calculate hive position as odometry (x2,y2) via AprilTags
        calculate distance from (x1,y1) to hive (x2,y2)
        given initial velocity (calculated from Auto), do math to generate RPM
        set motor speed to generated RPM
        shoot consecutively
         */
    }
}
