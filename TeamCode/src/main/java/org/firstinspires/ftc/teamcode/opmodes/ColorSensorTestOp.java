package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by michaelzhou on 10/26/16.
 */

@Autonomous(name = "ColorSensorTest", group = "Autonomous")
public class ColorSensorTestOp extends LinearOpMode {
    ColorSensor colorSensor;

    public void runOpMode() throws InterruptedException{
        waitForStart();

        //get and reset
        colorSensor = hardwareMap.colorSensor.get("colorSensor1");
        colorSensor.enableLed(false);

        while(opModeIsActive()){
            DbgLog.msg("red = %d, blue = %d, green = %d",colorSensor.red(),colorSensor.blue(),colorSensor.green());
            sleep(1000);
        }
        stop();
    }

}
