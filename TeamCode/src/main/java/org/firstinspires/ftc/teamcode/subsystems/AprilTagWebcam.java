package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    double pitchCutoff = 0;
    private AprilTagProcessor processor;
    private VisionPortal portal;

    private ArrayList<AprilTagDetection> detectedTags = new ArrayList<>();
    private Telemetry telemetry;
    public void init(HardwareMap hwMap, Telemetry telemetry, String name){
        this.telemetry = telemetry;
        processor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, name));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(processor);

        portal = builder.build();
    }

    public void update(){
        detectedTags = processor.getDetections();
    }
    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagSingleDetection getTagBySpecificID(int id){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagSingleDetection){
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                if(singleDet.id == id){
                    telemetry.addLine("Single ID: " + singleDet.id);
                    return singleDet;
                }
            }
        }
        return null;
    }
    public AprilTagClusterDetection getCluster(){
        for(AprilTagDetection detection : detectedTags){
            if(detection instanceof AprilTagClusterDetection){
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                telemetry.addLine("Cluster Name: " + clusterDet.metadata.name);
                telemetry.addLine("Cluster Orientation: " + clusterDet.metadata.fieldOrientation);
                return clusterDet;

            }
        }
        return null;
    }
    public AprilTagClusterDetection getCluster(AprilTagDetection detection){
        if(detection instanceof AprilTagClusterDetection){
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
            telemetry.addLine("Cluster Name: " + clusterDet.metadata.name);
            telemetry.addData("Cluster x", clusterDet.ftcPose.x);
            telemetry.addData("Cluster y", clusterDet.ftcPose.y);
            telemetry.addData("Cluster yaw", clusterDet.ftcPose.yaw);
            telemetry.addData("Cluster pitch", clusterDet.ftcPose.pitch);
            return clusterDet;
        }
        return null;
    }
    public boolean hiveUp(AprilTagDetection detection){
        if(detection instanceof AprilTagClusterDetection){
            AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
            return clusterDet.ftcPose.pitch >= pitchCutoff;
        }
        return false;
    }
    public void stop(){
        if(portal != null){
            portal.close();
        }
    }
}
