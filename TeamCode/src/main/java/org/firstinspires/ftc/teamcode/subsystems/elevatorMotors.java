package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class elevatorMotors {
    private  DcMotorEx left, right;
    public elevatorMotors(DcMotorEx left,DcMotorEx right){
        this.left = left;
        this.right = right;
    }
    public void init(HardwareMap hardwareMap){
        left = hardwareMap.get(DcMotorEx.class, "lEl");
        right = hardwareMap.get(DcMotorEx.class,"rEl");
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncoder(){
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPower(double l, double r){
        left.setPower(l);
        right.setPower(r);
    }
    public void setPower(double p){
        left.setPower(p);
        right.setPower(p);
    }
    public double getPosition(){
        return left.getCurrentPosition();
    }
    public void setRunMode(DcMotor.RunMode mode){
        left.setMode(mode);
        right.setMode(mode);
    }
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode){
        left.setZeroPowerBehavior(mode);
        right.setZeroPowerBehavior(mode);
    }
    public double getVelocity(){
        return left.getVelocity();
    }
    public void enable(){
        left.setMotorEnable();
        right.setMotorEnable();
    }
    public void disable(){
        left.setMotorDisable();
        right.setMotorDisable();
    }


}
