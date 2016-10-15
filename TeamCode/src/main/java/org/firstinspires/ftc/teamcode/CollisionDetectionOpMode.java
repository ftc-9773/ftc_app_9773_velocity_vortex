package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivesys.FourMotorTankDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;

/**
 * Created by michaelzhou on 9/30/16.
 */

@Autonomous(name = "Collision Detection", group = "Autonomous")
public class CollisionDetectionOpMode extends LinearOpMode {

    private final int NAVX_DIM_I2C_PORT = 0;
    //    private final double collisionThreshold = 0.5;
    double lastAccelX,lastAccelY;
    private AHRS navx_device;
    private OpticalDistanceSensor ods;
    private boolean collisionState;
    private ElapsedTime runtime = new ElapsedTime();

    private long sensor_timestamp_delta = 0;
    private long system_timestamp_delta = 0;

    DcMotor fMotorL;
    DcMotor fMotorR;
    DcMotor rMotorL;
    DcMotor rMotorR;

    double maxSpeed = 1.0;
    double minSpeed = 0.0;

    double frictionCoefficient = 1.0;

    FourMotorTankDrive fourMotorDrive;

    Wheel wheel;

    public void runOpMode() throws InterruptedException{
        //init()
        ods = (OpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods");
        DeviceInterfaceModule dim = null;
        dim = hardwareMap.deviceInterfaceModule.get("dim");
        navx_device = AHRS.getInstance(dim, NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        fMotorL = hardwareMap.dcMotor.get("fMotorL");
        fMotorR = hardwareMap.dcMotor.get("fMotorR");
        rMotorL = hardwareMap.dcMotor.get("rMotorL");
        rMotorR = hardwareMap.dcMotor.get("rMotorR");
        wheel = new Wheel(Wheel.Type.RUBBER_TREADED, 4.0);

        fourMotorDrive = new FourMotorTankDrive(fMotorL,fMotorR,rMotorL,rMotorR,maxSpeed,minSpeed,frictionCoefficient,wheel,1120);

        lastAccelX = 0.0; lastAccelY = 0.0; setCollision(false);

        fMotorR.setDirection(DcMotor.Direction.REVERSE);
        rMotorR.setDirection(DcMotor.Direction.REVERSE);



        waitForStart();
        DbgLog.msg("navX Op Init Loop", runtime.toString());
        while (opModeIsActive()){
            DbgLog.msg("1. NavX device = %s", navx_device.isConnected() ? "Connected" : "Not connected");
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
            DbgLog.msg("2. GyroAccel = %s", gyroCal);
            DbgLog.msg("3. Motion = %s", motion);
            DbgLog.msg("4. Collision = %s", getCollision());
            DbgLog.msg("5. Timing = %s", Long.toString(sensor_timestamp_delta) + ", " +
                    Long.toString(system_timestamp_delta));
            DbgLog.msg("6 Events = %s", Double.toString(navx_device.getUpdateCount()));

            
            
            fourMotorDrive.drive((float) 0.5,0);
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
