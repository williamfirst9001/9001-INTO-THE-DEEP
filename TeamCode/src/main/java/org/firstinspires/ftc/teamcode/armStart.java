package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.autoGetPoints.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.elevator;

public class armStart {
    static robotHardware robot = robotHardware.getInstance();
    static elevator arm = new elevator();
    static Wrist wrist = new Wrist();
    private static boolean zeroed = false;
    public static void start(){
        if(!zeroed){
            robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.elevatorMotor.setPower(-.75);
        }
        if(robot.armSwitch.isPressed()){
            robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeroed = true;
        }
        if(zeroed){
            arm.goToSetpoint(0);
        }

    }
    public static void reset(){
        zeroed = false;
    }
}
