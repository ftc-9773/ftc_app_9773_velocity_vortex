package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by michaelzhou on 9/30/16.
 */

@Autonomous(name = "Collision Detection", group = "Autonomous")
public class CollisionDetectionOp extends LinearOpMode {

    private final int NAVX_DIM_I2C_PORT = 0;
//    private final double collisionThreshold = 0.5;
    double lastAccelX,lastAccelY;
    private AHRS navx_device;
    private boolean collisionState;
    private ElapsedTime runtime = new ElapsedTime();

    private long sensor_timestamp_delta = 0;
    private long system_timestamp_delta = 0;

    public void runOpMode() throws InterruptedException{
        //init()
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"), NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        lastAccelX = 0.0; lastAccelY = 0.0; setCollision(false);

        waitForStart();
        telemetry.addData("navX Op Init Loop", runtime.toString());
        while (opModeIsActive()){
            telemetry.addData("1. NavX device", navx_device.isConnected() ? "Connected" : "Not connected");
            String gyroCal, motion;
            if(navx_device.isConnected()){
                gyroCal = (navx_device.isCalibrating() ? "Calibrating" : "Calibration completed");
                motion = (navx_device.isMoving() ? "Moving" : "Not Moving");
                if(navx_device.isRotating()){
                    motion += ", Rotating";
                }
            }
            else{
                gyroCal = motion = "------";
            }
            //setCollision(collision); ???
            telemetry.addData("2. GyroAccel", gyroCal);
            telemetry.addData("3. Motion", motion);
            telemetry.addData("4. Collision", getCollision());
            telemetry.addData("5. Timing", Long.toString(sensor_timestamp_delta) + ", " +
                    Long.toString(system_timestamp_delta));
            telemetry.addData("6 Events", Double.toString(navx_device.getUpdateCount()));
        }

        idle();
    }

    //collision state getter
    private String getCollision(){
        return collisionState ? "COLLISION" : "NO COLLISION";
    }

    //collision state setter - needed? or callback?
    private void setCollision(boolean newValue){
        this.collisionState = newValue;
    }
}
