package org.firstinspires.ftc.teamcode.drivesys;

import com.qualcomm.robotcore.hardware.DcMotor;

public class FourMotorDrive {
    DcMotor fMotorL = null;
    DcMotor fMotorR = null;
    DcMotor rMotorL = null;
    DcMotor rMotorR = null;
    double frictionCoefficientR;
    double frictionCoefficientL;
    double maxSpeed;
    double minSpeed;
    Wheel wheel;
    double[] wheelValues;

    public FourMotorDrive(DcMotor fMotorL, DcMotor fMotorR, DcMotor rMotorL, DcMotor rMotorR, double maxSpeed, double minSpeed, double frictionCoefficientR, double frictionCoefficientL, Wheel wheel){
        this.fMotorL = fMotorL;
        this.fMotorR = fMotorR;
        this.rMotorL = rMotorL;
        this.rMotorR = rMotorR;
        this.frictionCoefficientR = frictionCoefficientR;
        this.frictionCoefficientL = frictionCoefficientL;
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.wheel = wheel;
        this.wheelValues = wheel.getValues();
    }

    public void drive(float speed, float direction){
        double left = (speed - direction) * frictionCoefficientL;
        double right = (speed + direction) * frictionCoefficientR;

        fMotorL.setPower(left);
        fMotorR.setPower(right);
        rMotorL.setPower(left);
        rMotorR.setPower(right);
    }

    public void driveToDistance(float speed, float direction, double distance){
        double startingPositionfL = fMotorL.getCurrentPosition();
        double startingPositionfR = fMotorR.getCurrentPosition();

        double targetPosition =(distance / wheelValues[1]) * 1120;

        while(((fMotorL.getCurrentPosition()-startingPositionfL)<targetPosition) && ((fMotorR.getCurrentPosition()-startingPositionfR)<targetPosition)){
            drive(speed, direction);
        }
        fMotorR.setPower(0);
        fMotorL.setPower(0);
        rMotorR.setPower(0);
        rMotorL.setPower(0);
    }
}
