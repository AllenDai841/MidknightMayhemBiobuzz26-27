package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
        if (result == null || !result.isValid()){
            detectedList = new ArrayList<>();
        } else {
            detectedList = result.getDetectorResults();
        }
        return detectedList;
    }
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }
    public void start(){
        limelight.start();
    }
}
