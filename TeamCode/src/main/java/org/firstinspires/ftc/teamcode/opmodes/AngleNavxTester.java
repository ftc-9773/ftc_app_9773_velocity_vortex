package org.firstinspires.ftc.teamcode.opmodes;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.MatrixConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drivesys.FourMotorTankDrive;
import org.firstinspires.ftc.teamcode.drivesys.Wheel;
import org.firstinspires.ftc.teamcode.navigation.NavxMicro;

import java.security.Timestamp;

/**
 * Created by michaelzhou on 10/26/16.
 */

@TeleOp(name = "AngleNavxTest", group = "TeleOp")
public class AngleNavxTester extends LinearOpMode{
    DcMotor fMotorL;
    DcMotor fMotorR;
    DcMotor rMotorL;
    DcMotor rMotorR;
    FourMotorTankDrive drivesys;
    Wheel wheel;

    /* This is the port on the Core Device Interace Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.3;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.3;
    private final double YAW_PID_P = 0.05;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        fMotorL = hardwareMap.dcMotor.get("fMotorL");
        fMotorR = hardwareMap.dcMotor.get("fMotorR");
        rMotorL = hardwareMap.dcMotor.get("rMotorL");
        rMotorR = hardwareMap.dcMotor.get("rMotorR");
        wheel = new Wheel("rubber_treaded", 4);
        drivesys = new FourMotorTankDrive(fMotorL,rMotorL,fMotorR,rMotorR,1,0,1,wheel,2240);


        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

//        fMotorR.setDirection(DcMotor.Direction.REVERSE);

        /* If possible, use encoders when driving, as it results in more */
        /* predicatable drive system response.                           */

        /* Create a PID Controller which uses the Yaw Angle as input. */
        //yawPIDController = new navXPIDController( navx_device,
                //navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        /*yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);*/

        waitForStart();

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = 30.0;
        int DEVICE_TIMEOUT_MS = 500;
        //navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
        navx_device.zeroYaw();

        DbgLog.msg("Reached here 1!!");


        while (opModeIsActive() && (TARGET_ANGLE_DEGREES > Math.abs(navx_device.getYaw()))) {
            DbgLog.msg("Reached here 2!!");
            DbgLog.msg("yaw: %f", navx_device.getYaw());

            drivesys.lineFollow(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);

            idle();
        }
        drivesys.stop();
        stop();
    }
}

