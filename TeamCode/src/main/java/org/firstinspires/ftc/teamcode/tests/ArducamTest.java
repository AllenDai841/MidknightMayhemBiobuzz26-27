package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;

public class ArducamTest extends OpMode {
    AprilTagWebcam arducam;
    @Override
    public void init() {
        arducam = new AprilTagWebcam();
        arducam.init(hardwareMap, telemetry, "arducam");
    }

    @Override
    public void loop() {
        arducam.update();
        telemetry.addLine(arducam.getDetectedTags().toString());
        arducam.getCluster();
    }
}
