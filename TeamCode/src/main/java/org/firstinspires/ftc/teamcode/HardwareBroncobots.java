package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is not an opmode.  Here all the hardware related to the Broncobots is shown here.
 *         frontLeft  =  "front_left_motor");
 *
 *         frontRight =  "front_right_motor");
 *
 *         backLeft   = "back_left_motor");
 *
 *         backRight  = "back_right_motor");
 *
 */

public class HardwareBroncobots {


    /* Public OpMode members. */
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBroncobots{

    }

    /* initialize the hardware*/

    public void init(HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap = ahwMap;

        frontLeft  = hwMap.get(DcMotorEx.class, "front_left_motor");
        frontRight = hwMap.get(DcMotorEx.class, "front_right_motor");
        backLeft = hwMap.get(DcMotorEx.class, "back_left_motor");
        backRight = hwMap.get(DcMotorEx.class, "back_right_motor");

        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft .setPower(0);
        backRight.setPower(0);

        // Set all motors to run without encoders.
        // If you are planning to use PID.  If you do not want to use the PID, you can run with the encorder

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        For angle turn, we need these motors to reverse

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

         */
    }

}
