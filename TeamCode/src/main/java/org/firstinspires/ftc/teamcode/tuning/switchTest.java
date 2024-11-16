package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "switchTest", group = "Linear OpMode")
public class switchTest extends LinearOpMode {
    private TouchSensor sensor;
    public void runOpMode(){
        sensor = hardwareMap.get(TouchSensor.class, "pivotlimit");
        waitForStart();
        while(opModeIsActive()){
            if(sensor.isPressed()){
                telemetry.addData("pressed",true);
            } else{
                telemetry.addData("pressed",false);
            }
            telemetry.update();
        }
    }
}
