package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "claw test", group = "Linear OpMode")
public class clawTest extends LinearOpMode {
    private Servo claw;
    public void runOpMode(){
        claw = hardwareMap.get(Servo.class,"claw");
        waitForStart();
        while(opModeIsActive()){
    if(gamepad1.x){
        claw.setPosition(clawLimits.open);
    }
    if(gamepad1.y){
        claw.setPosition(clawLimits.close);
    }


        }
    }
}
