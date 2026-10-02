package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public class LimelightTest extends OpMode {
    Limelight limelight;
    @Override
    public void init() {
        limelight = new Limelight(hardwareMap.get(Limelight3A.class, "limelight"));
        limelight.start();
    }

    @Override
    public void loop() {
        limelight.getResult();
        limelight.getDetected();
        telemetry.addLine("Detected: "+ limelight.getDetected());
    }
}
