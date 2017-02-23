package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.vision.Vision;

/**
 * Created by pranavburugula on 2/22/2017.
 */

@TeleOp(name = "VisionTest", group = "TeleOp")
public class VisionTest extends LinearOpMode {
    Vision vision = null;

    @Override
    public void runOpMode(){
        vision = new Vision(this);

        waitForStart();
        while (opModeIsActive()){
            int color = vision.getBeaconColor();

            telemetry.addData("color:", color);
            telemetry.update();
            DbgLog.msg("Color: %d", color);
            idle();
        }
    }
}
