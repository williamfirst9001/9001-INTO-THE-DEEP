package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "wrist test", group = "Linear OpMode")
public class wristTest extends LinearOpMode {
    private Servo wrist;
    public void runOpMode(){
        wrist = hardwareMap.get(Servo.class,"wristServo");
        waitForStart();
        while(opModeIsActive()){
    if(gamepad1.x){
        wrist.setPosition(wristLimits.top);
    }
    if(gamepad1.y){
        wrist.setPosition(wristLimits.bottom);
    }
    if(gamepad1.a){
        wrist.close();
    }
    if(gamepad1.b){
        wrist = hardwareMap.get(Servo.class,"wristServo");
    }
        }
    }
}
