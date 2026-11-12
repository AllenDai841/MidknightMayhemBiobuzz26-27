package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilcamTest extends OpMode {
    AprilTagWebcam aprilcam;
    @Override
    public void init() {
        aprilcam = new AprilTagWebcam();
        aprilcam.init(hardwareMap, telemetry, "webcam");
    }

    @Override
    public void loop() {
        telemetry.addData("running", 1);
        aprilcam.update();
        if(!aprilcam.getDetectedTags().isEmpty()){
            for(AprilTagDetection detection : aprilcam.getDetectedTags()){
                aprilcam.getCluster(detection);
            }
        }
    }
}
