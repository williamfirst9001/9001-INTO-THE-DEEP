package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "wrist test", group = "Linear OpMode")
public class wristTest extends LinearOpMode {
    private Servo wrist;
    public void runOpMode(){
        wrist = hardwareMap.get(Servo.class,"wristservo");
        waitForStart();
        while(opModeIsActive()){
    if(gamepad1.x){
        wrist.setPosition(wristLimits.pickUp);
    }
    if(gamepad1.y){
        wrist.setPosition(wristLimits.stow);
    }
    if(gamepad1.b){
        wrist.setPosition(wristLimits.basket);
    }
    if(gamepad1.a){
        wrist.setPosition(wristLimits.specimen);
    }
        }
    }
}
