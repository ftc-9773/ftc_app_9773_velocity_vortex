package org.firstinspires.ftc.teamcode.opmodes;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import org.firstinspires.ftc.teamcode.navigation.NavxMicro;

/**
 * Created by michaelzhou on 10/26/16.
 */

@TeleOp(name = "NavxTest", group = "TeleOp")
public class NavxTestOp extends LinearOpMode {
    private AHRS navxDevice;
    private final int NAVX_DIM_I2C_PORT = 0;

    public void runOpMode(){
        DeviceInterfaceModule dim;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        navxDevice = AHRS.getInstance(dim, NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        waitForStart();

        while (opModeIsActive()){
            DbgLog.msg("pitch = %f, yaw = %f, roll = %f",navxDevice.getPitch(), navxDevice.getYaw(), navxDevice.getRoll());
            sleep(1000);
        }
        stop();

    }
}
