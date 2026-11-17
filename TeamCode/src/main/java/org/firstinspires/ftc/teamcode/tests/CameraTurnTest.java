package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.Motor;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class CameraTurnTest extends OpMode {
    String tagName = "RED SCORING";
    AprilTagWebcam aprilcam;
    Motor turner;
    public static double ticksPerRev = 28;
    AprilTagClusterDetection clusterDet;
    double constant = 0.1;
    boolean targetFound = false;

    @Override
    public void init() {
        aprilcam = new AprilTagWebcam();
        aprilcam.init(hardwareMap, telemetry, "webcam");
        turner = new Motor(hardwareMap.get(DcMotorEx.class, "turner"), ticksPerRev);
    }

    @Override
    public void loop() {
        telemetry.addData("running", 1);
        aprilcam.update();
        if(!aprilcam.getDetectedTags().isEmpty()){
            for(AprilTagDetection detection : aprilcam.getDetectedTags()){
                clusterDet = aprilcam.getCluster(detection);
                if(clusterDet != null && clusterDet.metadata.name.equals(tagName)){
                    targetFound = true;
                    if(aprilcam.getErrorXCluster(clusterDet) * constant >= 0.5){
                        turner.setPower(0.5);
                    }
                    else if(aprilcam.getErrorXCluster(clusterDet) * constant <= -0.5){
                        turner.setPower(-0.5);
                    }
                    else if(aprilcam.getErrorXCluster(clusterDet) * constant < 0.3 && aprilcam.getErrorXCluster(clusterDet) * constant > 0){
                        turner.setPower(0.3);
                    }
                    else if(aprilcam.getErrorXCluster(clusterDet) * constant > -0.3 && aprilcam.getErrorXCluster(clusterDet) * constant < 0){
                        turner.setPower(-0.3);
                    }
                    else{
                        turner.setPower(aprilcam.getErrorXCluster(clusterDet) * constant);
                    }
                    break;
                }
            }
        }
        if(!targetFound){
            turner.setPower(0);
        }
        targetFound = false;
    }
}
