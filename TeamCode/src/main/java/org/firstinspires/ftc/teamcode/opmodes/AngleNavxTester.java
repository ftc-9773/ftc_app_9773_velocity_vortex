package org.firstinspires.ftc.teamcode.opmodes;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by michaelzhou on 10/26/16.
 */

public class AngleNavxTester extends LinearOpMode{
    DcMotor fMotorL;
    DcMotor fMotorR;
    DcMotor rMotorL;
    DcMotor rMotorR;

    /* This is the port on the Core Device Interace Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        fMotorL = hardwareMap.dcMotor.get("fMotorL");
        fMotorR = hardwareMap.dcMotor.get("fMotorR");
        rMotorL = hardwareMap.dcMotor.get("rMotorL");
        rMotorR = hardwareMap.dcMotor.get("rMotorR");

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);

        fMotorR.setDirection(DcMotor.Direction.REVERSE);

        /* If possible, use encoders when driving, as it results in more */
        /* predicatable drive system response.                           */
        fMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
       // yawPIDController.setTolerance(TimestampedPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

        waitForStart();

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        final double TOTAL_RUN_TIME_SECONDS = 30.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        while ( runtime.time() < TOTAL_RUN_TIME_SECONDS ) {
            if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                if ( yawPIDResult.isOnTarget() ) {
                    fMotorL.setPower(fMotorL.getPower());
                    fMotorR.setPower(fMotorR.getPower());
                    rMotorL.setPower(rMotorL.getPower());
                    rMotorR.setPower(rMotorR.getPower());
                } else {
                    double output = yawPIDResult.getOutput();
                    if ( output < 0 ) {
                        /* Rotate Left */
                        fMotorL.setPower(-output);
                        fMotorR.setPower(output);
                        rMotorL.setPower(-output);
                        rMotorR.setPower(output);
                    } else {
                        /* Rotate Right */
                        fMotorL.setPower(output);
                        fMotorR.setPower(-output);
                        rMotorL.setPower(output);
                        rMotorR.setPower(-output);
                    }
                }
            } else {
			          /* A timeout occurred */
                DbgLog.msg("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        }
    }
}
}
