package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

class limeLight {
    private HashMap<Integer,String> pipeLineMap = new HashMap<>();
    private HardwareMap hardwareMap;
    private Limelight3A limelight;
    private int curPipeline = 0;
    private boolean isRunning = true;
    public limeLight(HardwareMap hardwareMap){
            this.hardwareMap = hardwareMap;
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.start(); // This tells Limelight to start looking!
            limelight.pipelineSwitch(0); // Switch to pipeline number 0

            pipeLineMap.put(0,"objDetect");
            pipeLineMap.put(1,"aprilTag");
    }
    public void setPollRate(int pollRate){
        limelight.setPollRateHz(pollRate);
    }
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
        curPipeline = pipeline;
    }
    public LLResult getObjResults(){
        LLResult results = limelight.getLatestResult();
        if (results.isValid()){
        if(pipeLineMap.get(curPipeline)=="objDetect"){
            return results;
        } else return null;
        } else return null;
        }

    public LLResult getFidResults(){
        LLResult results = limelight.getLatestResult();
        if (results.isValid()){
        if(pipeLineMap.get(curPipeline)=="aprilTag") {
            return results;
        } else return null;
        } else return null;

    }

    

}
