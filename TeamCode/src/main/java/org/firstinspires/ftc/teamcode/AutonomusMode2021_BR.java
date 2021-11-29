/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BR Autonomus Mode", group = "ftc16671")
@Disabled
public class AutonomusMode2021_BR extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    /** This is for encoder **/
    static final double     COUNTS_PER_MOTOR_REV    = 500 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    //
    static final int oneRingMinHeight = 180;
    static final int oneRingMaxHeight = 240;

    static final int fourRingsMinHeight = 240;
    static final int fourRingsMaxHeight = 340;

    static final int SECONDS_TO_DETECT_OBJECT = 0;

    private MecanumDrive mecanumDrive = new MecanumDrive();
    private ElapsedTime runtime = new ElapsedTime();
    public MecanumDrive mDrive = new MecanumDrive();

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     */
    private static final String VUFORIA_KEY =
            "AWrb413/////AAABmQT6xsY2eEeEuRH7ulHkqXaAxt2nbyCB1ZDQfx1F+X80Nz5JjPzStB+GpmAByIBVfrjDCkRdsHsurFZvZruc+Rr8KeaKixYFNtpkbmk9DxNPtR3Tq67CVKTZYC46SR+zghr8zn5nP9NLOHWcVYFcNuTR8rx7R9QzAPlKYX60OHC6OLc5ngylJH/zvESjkSMq/84O68lIfkKVycJ7a8085IQBGfVh/yYEJQg3txuehOK97yTSltcJ8CYiM0qZBVRtGIbS2N6D8IZc8BpyjqTaZ8YZhE2gjCYVtlBKk6pveRidtkb0UA1uVmaVR0B9FeSlzwx8h38nbnIJlJF/WOuXNSApPALRl5wn8FZuY01VnV0s";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //This is for mobile phone
        //parameters.cameraDirection = CameraDirection.BACK;
        //This is for external camera - MAKE SURE DEVICE NAME IS CORRECT

        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }



    @Override
    public void runOpMode() throws InterruptedException {

        //initVuforia();
        //initTfod();
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initCarousel_and_lift(hardwareMap);
        mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntake(hardwareMap);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.2, 1.78);
            //tfod.setZoom(1, 16.0/9.0);
        }
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        long start = System.currentTimeMillis();
        long end = System.currentTimeMillis();
        int totalRings = 0;
        if (opModeIsActive()) {

            while (opModeIsActive()) {
                //if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    float seconds = (end - start) / 1000F;
                    /*
                    while(seconds <= SECONDS_TO_DETECT_OBJECT) {
                        telemetry.addData("# seconds passed : ", seconds);
                        telemetry.update();
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        end = System.currentTimeMillis();
                        seconds = (end - start) / 1000F;
                        if (updatedRecognitions != null) {

                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            telemetry.addData("# seconds passed : ", seconds);

                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("Total rings height (%d)", i), "%.03f", recognition.getHeight());
                                telemetry.update();
                                float totalHeight = recognition.getHeight();

                            }
                            telemetry.update();

                        } else {
                            tfod.shutdown();
                        }
                    }*/
                    // For RED1, with front against wall
                     // Identify location of shipping element/ducky
                // For RED1, with front against wall
                // Identify location of shipping element/ducky
                mecanumDrive.box.setPosition(.90);
                //lift up
                mecanumDrive.moveLiftUp(500, 0.4);
                //Move backward
                //mecanumDrive.moveBackward(3,true,5,0.4, telemetry);
                //mecanumDrive.rotateLeftSide(3, true,5,0.4,telemetry);
                //mecanumDrive.strafeRight(5, true,5, 0.4, telemetry);
                //mecanumDrive.moveBackward(1,true,5,0.4,telemetry);
                //mecanumDrive.runCarousel(0.4);
                //sleep(5000);
                //mecanumDrive.runCarousel(0);
               // mecanumDrive.rotateLeftSide(7, true, 5, 0.4, telemetry);
                sleep(500);
                mecanumDrive.box.setPosition(.66);
                //commit does not work
                mecanumDrive.moveBackward(5, true,5, 0.4, telemetry);//7
                //Strafe Right
                mecanumDrive.strafeRight(10,true,5, 0.4, telemetry);
                //Move backward (make sure to contact the shipping hub)
                mecanumDrive.moveBackward(3, true, 5, 0.2, telemetry);//1
                //Move lifter as
                mecanumDrive.moveLiftUp(1600, 0.4);
                // recognized by Vuforia and based on location of duck
                //Use dropping function
                sleep(750);//2000
                //wait for two seconds
                mecanumDrive.dumpAndBringbackBox();
                //just to commit
                //just to commit
                // Move forwaard slightly
                mecanumDrive.moveForward(6,true,5,0.4,telemetry);//5
                //move lift down
                mecanumDrive.moveLiftUp(500,0.4);
                //turn left 90 degrees
                mecanumDrive.rotateRightSide(7.15,true,5,0.4,telemetry);//3
                //Go foward and park in the warehouse
                mecanumDrive.moveBackward(16,true,5,0.4,telemetry);//12, 18, 7.2
                mecanumDrive.moveBackward(2.5, true, 5, 0.2,telemetry);//Two sets of moving backwards at different speeds so that the robot doesn't bounce
               // mecanumDrive.strafeRight(3,true,5,0.4, telemetry);
                mecanumDrive.runCarousel(0.4);
                sleep(5000);
                mecanumDrive.strafeRight(8,true, 5,0.4,telemetry);
                mecanumDrive.moveBackward(2, true, 5, 0.4, telemetry);
                //puting box back to floor to prepare for tele op
                mecanumDrive.box.setPosition(1);
                sleep(750);
                mecanumDrive.moveLiftUp(0,0.4);
                break;

                //}
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}
