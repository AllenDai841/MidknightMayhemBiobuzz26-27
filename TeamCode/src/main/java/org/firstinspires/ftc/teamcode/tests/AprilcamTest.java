package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;

public class AprilcamTest extends OpMode {
    AprilTagWebcam aprilcam;
    @Override
    public void init() {
        aprilcam = new AprilTagWebcam();
        aprilcam.init(hardwareMap, telemetry, "aprilcam");
    }

    @Override
    public void loop() {
        aprilcam.update();
        telemetry.addLine(aprilcam.getDetectedTags().toString());
        aprilcam.getCluster();
    }
}
