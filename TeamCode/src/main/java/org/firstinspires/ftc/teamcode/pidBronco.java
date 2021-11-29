package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


public class pidBronco extends LinearOpMode{
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    private BNO055IMU imu;




    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU ;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize((parameters));
/*
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hwMap.get(DcMotor.class, "back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
        waitForStart();
        double referenceAngle = Math.toRadians(90);

        while (opModeIsActive()) {
            //double power = PIDControl(100, motor.getCurrentPosition());
            //motor.setPower(power);
            double power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle);
            frontLeft.setPower(power);
            backLeft.setPower(power);
            frontRight.setPower(-power);
            backRight.setPower(-power);
        }

    }

    public double PIDControl(double reference, double state){
        double error = angleWrap(reference - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;


        timer.reset();

        double ouput = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return ouput;

    }
    public double angleWrap(double radians) {
        while ((radians > Math.PI)) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }




}
