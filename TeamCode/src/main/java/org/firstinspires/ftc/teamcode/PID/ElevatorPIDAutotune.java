package org.firstinspires.ftc.teamcode.PID;

public class ElevatorPIDAutotune {


/**
    public List<Double> PIDTune(double angle) {
        List<Double> returnVals = new ArrayList<>();
        double growthVal = 0;
        double kP = constants.armPID.kP;
        double kI = constants.armPID.kI;
        double kD = constants.armPID.kD;
        double angleError;
        angleError = 1/90*Math.pow(angle-90,2)+91;


        growthVal = Math.pow((1 + constants.armPID.angleGrowthRate), angleError);

        kP *= constants.armPID.kPConstantMult + growthVal;
        kI *= constants.armPID.kIConstantMult + growthVal;
        kD *= constants.armPID.kDConstantMult + growthVal;
        returnVals.add(kP);
        returnVals.add(kI);
        returnVals.add(kD);


        return returnVals;
    }
**/

}
