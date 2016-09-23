package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by michaelzhou on 9/23/16.
 */
@TeleOp(name = "TestOpMike", group = "TeleOp")
public class TestOpModeMike extends LinearOpMode{
  //init
    DcMotor motor1 = null;
    DcMotor motor2 = null;

    public void runOpMode() throws InterruptedException{
      //get
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        //ready position for start
        waitForStart();

        //motors respond to button presses
        while(opModeIsActive()){
            if(gamepad1.a){
                motor1.setPower(1);
            }
            else if(gamepad1.y){
                motor1.setPower(-1);
            }
            else if(gamepad1.x){
                motor2.setPower(-1);
            }
            else if(gamepad1.b){
                motor2.setPower(1);
            }

            //delay 20ms and send signals
            idle();
        }
    }
}
