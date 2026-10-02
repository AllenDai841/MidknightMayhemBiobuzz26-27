package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;

public class Limelight{
    Limelight3A limelight;
    LLResult result;
    List<LLResultTypes.DetectorResult> detectedList;
    public Limelight(Limelight3A limelight){
        this.limelight = limelight;
    }
    public LLResult getResult(){
        result = limelight.getLatestResult();
        return result;
    }
    public List<LLResultTypes.DetectorResult> getDetected(){
        detectedList = result.getDetectorResults();
        return detectedList;
    }
    public void start(){
        limelight.start();
    }
}
