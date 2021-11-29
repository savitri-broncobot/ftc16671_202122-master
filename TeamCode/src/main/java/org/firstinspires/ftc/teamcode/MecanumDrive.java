 package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

 public class MecanumDrive {

     DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotor lift;
    DcMotor carousel;

    DcMotor intake;
    DcMotor conveyor;

    Servo holder;//holder for capstone
    Servo box;//box to hold cubes and balls

     private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

     private static final String[] LABELS = {
             "Ball",
             "Cube",
             "Duck",
             "Marker"
    };
//LABELS
    private static final String LABEL_DUCK = "Duck";
     private static final String LABEL_MARKER = "Marker";



    private double fast = 1.0; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.7; // medium speed

    private double slow = 0.4; // slow speed
    private double verySlow = 0.05; //very slow speed

    public static double GEAR_RATIO = 1.0; // for simulator - ours should be 0.5f;
    public static double WHEEL_RADIUS = 5.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 1120.0;  // From NeveRest (for simulator)  GoBilda should be 383.6f
    public static double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    /** This is for encoder **/
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double maxSpeed = 1.0;

    // drive motor position variables
    private int flPos; private int frPos; private int blPos; private int brPos;

    private MatrixF conversion;
    private GeneralMatrixF encoderMatrix = new GeneralMatrixF(3, 1);

    private int frontLeftOffset;
    private int frontRightOffset;
    private int backRightOffset;
    private int backLeftOffset;

    ElapsedTime runtime = new ElapsedTime();

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
     protected TFObjectDetector tfod;


     /**
      * Initialize the Vuforia localization engine.
      */
     protected void initVuforia(HardwareMap hardwareMap) {
         /*
          * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
          */
         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

         parameters.vuforiaLicenseKey = VUFORIA_KEY;
         parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

         //  Instantiate the Vuforia engine
         vuforia = ClassFactory.getInstance().createVuforia(parameters);

         // Loading trackables is not necessary for the TensorFlow Object Detection engine.
     }

     /**
      * Initialize the TensorFlow Object Detection engine.
      */
     protected void initTfod(HardwareMap hardwareMap) {
         int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                 "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
         tfodParameters.minResultConfidence = 0.8f;
         tfodParameters.isModelTensorFlow2 = true;
         tfodParameters.inputSize = 320;
         tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
         tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
     }


     public MecanumDrive() {

        float[] data = {1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, 1.0f};
        conversion = new GeneralMatrixF(3, 3, data);
        conversion = conversion.inverted();
    }

    void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hwMap.get(DcMotor.class, "back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //grabber = hwMap.get(Servo.class, "left_hand");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * Initialize all servos here.
     *
     * @param hwMap
     */
    void initServo(HardwareMap hwMap){
        box = hwMap.get(Servo.class, "box");//0 - control hub
        holder = hwMap.get(Servo.class, "holder");//0 - control hub
    }

    void initCarousel_and_lift(HardwareMap hwMap){
        lift = hwMap.get(DcMotor.class, "lifter");//
        carousel = hwMap.get(DcMotor.class, "carousel");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    void initIntake(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
    }

    void setAllMotorsToRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setAllMotorsToRunUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = maxSpeed;

        frontLeft.setPower(flSpeed);
        frontRight.setPower(frSpeed);
        backLeft.setPower(blSpeed);
        backRight.setPower(brSpeed);
    }

    void driveMecanum(double forward, double strafe, double rotate) {
        //watch following site to get more details on different options on how to move motors
        //https://stemrobotics.cs.pdx.edu/node/4746

        double frontLeftSpeed = forward - strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward + strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    double[] getDistanceCm() {
        double[] distances = {0.0, 0.0};

        encoderMatrix.put(0, 0, (float) ((frontLeft.getCurrentPosition() - frontLeftOffset) * CM_PER_TICK));
        encoderMatrix.put(1, 0, (float) ((frontRight.getCurrentPosition() - frontRightOffset) * CM_PER_TICK));
        encoderMatrix.put(2, 0, (float) ((backLeft.getCurrentPosition() - backLeftOffset) * CM_PER_TICK));

        MatrixF distanceMatrix = conversion.multiplied(encoderMatrix);
        distances[0] = distanceMatrix.get(0, 0);
        distances[1] = distanceMatrix.get(1, 0);

        return distances;
    }

    void setMaxSpeed(double speed) {
        maxSpeed = Math.min(speed, 1.0);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setEncoderOffsets() {
        frontRightOffset = frontRight.getCurrentPosition();
        frontLeftOffset = frontLeft.getCurrentPosition();
        backLeftOffset = backLeft.getCurrentPosition();
        backRightOffset = backRight.getCurrentPosition();
    }

    public void setAllWheelsToTargetPosition(int distance) {
        frontLeft.setTargetPosition((int) (distance*COUNTS_PER_INCH));
        frontRight.setTargetPosition((int) (distance*COUNTS_PER_INCH));
        backLeft.setTargetPosition((int) (distance*COUNTS_PER_INCH));
        backRight.setTargetPosition((int) (distance*COUNTS_PER_INCH));
    }

    public void moveForward(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos += distanceInInches * COUNTS_PER_INCH;
        frPos += distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Moving forward...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);


    }

     public void moveForward(double distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

         //get current position for all motors so we can start from there
         flPos = frontLeft.getCurrentPosition();
         frPos = frontRight.getCurrentPosition();
         blPos = backLeft.getCurrentPosition();
         brPos = backRight.getCurrentPosition();

         // calculate new targets
         flPos += distanceInInches * COUNTS_PER_INCH;
         frPos += distanceInInches * COUNTS_PER_INCH;
         blPos += distanceInInches * COUNTS_PER_INCH;
         brPos += distanceInInches * COUNTS_PER_INCH;

         //now since we have right positions for all motors, set target positions for all motors
         frontLeft.setTargetPosition(flPos);
         frontRight.setTargetPosition(frPos);
         backLeft.setTargetPosition(blPos);
         backRight.setTargetPosition(brPos);

         setAllMotorsToRunToPosition();
         runtime.reset();
         setSpeeds(speed, speed,speed,speed);
         while (runtime.seconds() < timeoutS &&
                 (frontLeft.isBusy() && frontRight.isBusy())) {
             //wait or print something in telemetry
             telemetry.addLine("Moving forward...");
             telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
             telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                     frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                     backRight.getCurrentPosition());
             //telemetry.update();
         }
         setSpeeds(0,0,0,0);


     }

    /**
     * Move specific distance (in inches) backwards with specific speed.     *
     *
     * @param distanceInInches
     * @param isOpModeActive
     * @param timeoutS
     * @param speed
     * @param telemetry
     */
    public void moveBackward(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos -= distanceInInches * COUNTS_PER_INCH;
        brPos -= distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Moving backward...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    /**
     * Move specific distance (in inches) backwards with specific speed.     *
     *
     * @param distanceInInches
     * @param isOpModeActive
     * @param timeoutS
     * @param speed
     * @param telemetry
     */
    public void moveBackward(double distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos -= distanceInInches * COUNTS_PER_INCH;
        brPos -= distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Moving backward...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void strafeLeft(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Strafing left..");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void strafeRight(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Strafing right...");
            telemetry.addData("runtime seconds ",runtime.seconds());
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void rotateRightSide(double distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos += distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos -= distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Rotating left..");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void rotateLeftSide(double distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos += distanceInInches * COUNTS_PER_INCH;
        blPos -= distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Rotating right...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            //telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }


    public void moveBasedOnTotalRings(int totalRings, Telemetry telemetry) {
        //First step drive 3 inches foward
        moveForward(18, true, 5, fast, telemetry);
        //strafe left 21 inches
        shootPowerShots(.526);
        sleep(900);
        strafeLeft(24, true, 5,slow,telemetry);
        //lift and shoot ring power shot new function
        pushRingForwardBack();
        //strafe 8 more inches
        strafeLeft(8, true, 5,slow,telemetry);
        //lift and shoot ring power shot
        sleep(400);
        pushRingForwardBack();
        //strafe 7 inches
        strafeLeft(8, true, 5,slow,telemetry);
        //Lift and shoot power shot
        sleep(400);
        pushRingForwardBack();
        sleep(400);
        // Stoping the shooter motors
        runShooterBack(0);
        runShooterFront(0);
        // Pushing lifter down
        moveLifter(.599);



        if(totalRings == 0){
            //Strafe left to B
            //strafeLeft(15, true, 5, fast, telemetry);
            //Move forward to A
            moveForwardAndRightBasedOnRings(totalRings, 24, 61, telemetry);

        }else if(totalRings == 1){
            //Strafe right
            //rotateRight(3, true, 5, slow, telemetry);
            //Strafe left to B
            moveForwardAndRightBasedOnRings(totalRings, 40, 42, telemetry);
            //
            //putWobbelArmDown();
            //
            //releaseWobble();
            //
            //moveBackward(10, true, 5, fast, telemetry);
            //
            //putWobbelArmUp();

        }else if(totalRings == 4){
            //Strafe right
            //strafeLeft(15, true, 5, fast, telemetry);
            //Move forward to A
            moveForwardAndRightBasedOnRings(totalRings, 57, 61, telemetry);
            //
            //putWobbelArmDown();
            //
            //releaseWobble();
            //
            //moveBackward(10, true, 5, fast, telemetry);
            //
            //putWobbelArmUp();

        }
    }

    public void moveBasedOnTotalRingsForTowerGoal(int totalRings, Telemetry telemetry) {
        //setting pusher position
        //pusher.setPosition(0.64);
        //running intake backwards to spread out rings
        if (totalRings == 4){
            runIntake(0.7);
        }
        runShooterFront(1);
        runShooterBack(1);
        //First step drive 3 inches foward
        moveForward(26, true, 5, medium, telemetry);
        //straif left 21 inches
        rotateRightSide(1,true,5,fast,telemetry);//0.8-0.9-1
        shootPowerShots(0.516);//.526-497-499-.498-496-512-517-516
        sleep(1500);
        if (totalRings == 4){
            runIntake(0);
        }
        //strafeLeft(24, true, 5,slow,telemetry);
        //lift and shoot ring power shot new function
        pushRingForwardBack();
        //straif 8 more inches
        //strafeLeft(8, true, 5,slow,telemetry);
        //lift and shoot rinconng power shot
        sleep(400);
        pushRingForwardBack();
        //straif 7 inches
        //strafeLeft(8, true, 5,slow,telemetry);
        //Lift and shoot power shot
        sleep(400);
        pushRingForwardBack();
        sleep(400);
        moveLifter(.599);
        //rotaterleft(2,true,5,fast,telemetry);
       // runIntake(1);
        if(totalRings == 0 ) {
            // Stoping the shooter motors
            runShooterBack(0);
            runShooterFront(0);
            // Pushing lifter down
            moveLifter(.599);
        }



        if(totalRings == 0){
            //Strafe left to B
            //strafeLeft(15, true, 5, fast, telemetry);
            //Move forward to A
            moveForwardAndRightBasedOnRings(totalRings, 16, 32, telemetry);//15-17-15-16
            //rotateRight(1, true, 5, slow, telemetry);
        }else if(totalRings == 1){
            runConveyor(-1.0);
            runIntake(-1.0);
            //starting intake to pick up extra ring
            moveForward(7, true,5, medium, telemetry);
            sleep(3500);
            runIntake(0);
            runConveyor(0);
            moveLifter(0.520);
            sleep(1000);
            pushRingForwardBack();
            // Stoping the shooter motors
            runShooterBack(0);
            runShooterFront(0);
            // Pushing lifter down
            moveLifter(.599);
            //Strafe right
            //rotateRight(1.2, true, 5, slow, telemetry);
            //Strafe left to B
           // moveForwardAndRightBasedOnRings(totalRings, 40, 7, telemetry);
            //strafeRightMoveForwardBasedOnRings(totalRings, 24, 25, telemetry);
            rotateLeftSide(1.1,true,5,fast,telemetry);//0.9-1.1
            moveForward(24, true,10, medium, telemetry);
            //
           // rotateRight(4.2, true, 5, slow, telemetry);
            //
            //putWobbelArmDown();
            //
            //releaseWobble();
            //
            //moveBackward(10, true, 5, fast, telemetry);
            //
            //putWobbelArmUp();

        }else if(totalRings == 4){
            //sleep(1000);

            runConveyor(-1.0);
            runIntake(-1.0);
            //starting intake to pick up extra ring
            moveForward(18, true,5, verySlow, telemetry);
            sleep(4000);
            runIntake(0);
            runConveyor(0);
            moveLifter(0.520);
            rotateRightSide(0.8,true, 5, fast, telemetry);
            sleep(2000);
            pushRingForwardBack();
            sleep(500);
            pushRingForwardBack();
            sleep(500);
            pushRingForwardBack();
            // Stoping the shooter motors
            runShooterBack(0);
            runShooterFront(0);
            // Pushing lifter down
            moveLifter(.599);
            //rotateRight(1,true,5,fast,telemetry);
            //Strafe right
            //strafeLeft(15, true, 5, fast, telemetry);
            //Move forward to A
            //moveForwardAndRightBasedOnRings(totalRings, 33, 26, telemetry);


            strafeRightMoveForwardBasedOnRings(totalRings, 38, 36, telemetry);
            //rotate
            rotateLeftSide(10,true,5,fast,telemetry);


        }
    }

    /**
     * After the robot has place the wobble in the corresponding square, the robot moves to the line from wherever it previously was.
     * It was able to do this based on total ring there were.
     *
     * @param totalRings -
     * @param telemetry
     */
    public void parkOnLineBasedOnRings(int totalRings, Telemetry telemetry){
        if(totalRings==0){
            double armPosition = .8;
            sleep(500);
            //Strafe left about 18 inches
            strafeLeft(33,true,5,fast,telemetry);
            //Move forward 8 inches, to white line
            moveForward(7,true,5,fast,telemetry);
        }else if (totalRings==1){
            double armPosition = .8;
            sleep(0);
            //Move backward 6 inches
            moveBackward(8, true,5,fast,telemetry);
        }else if (totalRings==4){
            telemetry.addLine("strafing left and moving backward");
            telemetry.update();
            strafeLeft(0, true, 5, fast, telemetry);
            //Move backward 30 inches
            moveBackward(31,true,5,fast,telemetry);
            double armPosition = .8;
            sleep(0);
        }
    }
    public void moveForwardAndRightBasedOnRings(int totalRings, int autoForward, int autoRight, Telemetry telemetry){
        moveForward(autoForward, true, 5, fast, telemetry);
        //Strafe left to
        strafeRight(autoRight, true, 5, fast, telemetry);
    }
    public void strafeRightMoveForwardBasedOnRings(int totalRings, int autoForward, int autoRight, Telemetry telemetry){

        //Strafe left to
        strafeRight(autoRight, true, 5, fast, telemetry);
        moveForward(autoForward, true, 5, fast, telemetry);
    }

    public void grabWobble(){
        double grabberPosition = 0.0;
    }

    public void releaseWobble(){
        double grabberPosition = 0.4;//0.4-0.25
        //grabber.setPosition(grabberPosition);
        sleep(0);
    }

    public void putWobbelArmDown(){
        double armPosition = 0.2;
        //wobbleArm.setPosition(armPosition);
        sleep(2000);
    }

    public void putWobbelArmUp(){
        double armPosition = .35;
        //wobbleArm.setPosition(armPosition);
        sleep(0);
    }



    public void runIntake(double intakePower){
        intake.setPower(intakePower);
    }

    public void runConveyor(double conveyorPower){
        conveyor.setPower(conveyorPower);
    }

    public void runShooterFront(double shooterFrontPower){
        //shooterFront.setPower(shooterFrontPower);
    }

    public void runShooterBack(double shooterBackPower){
        //shooterBack.setPower(shooterBackPower);
    }

    public void moveWobbleArmUp() {
        //wobbleArm.setPosition(0);
    }
    public void moveWobbleArmDown() {
        //wobbleArm.setPosition(1.0);
    }
    public void moveLifter() {
        //lifter.setPosition(0);
    }
    public void moveLifter(double lifterPosition) {
        //lifter.setPosition(lifterPosition);
    }
    public void moveLifterDown() {
        //lifter.setPosition(1.0);
    }


    public void liftUp() {
        //lifter.setDirection(Servo.Direction.FORWARD);
        //lifter.setPosition(0.5);
    }
    public void liftDown() {
        //lifter.setDirection(Servo.Direction.FORWARD);
        //lifter.setPosition(0.5);
    }

    /**
     * This method puts the current thread to sleep for the given time in msec.
     * It handles InterruptException where it recalculates the remaining time
     * and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param sleepTime specifies sleep time in msec.
     */
    public static void sleep(long sleepTime){
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0){
            try{
                Thread.sleep(sleepTime);
            }catch (InterruptedException e){

            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }

    public void putWobbleDownUp(){
        // Put the arm down
        putWobbelArmDown();
        // Open the fingers
        releaseWobble();
        // Lift the arm back up
        putWobbelArmUp();
    }

    public void putSecondWobbleDownUp(int totalRings, Telemetry telemetry){
       if (totalRings == 0){
           //stopping intake
           runIntake(0);
           //strafe left 25"
           strafeLeft(25,true, 5, fast, telemetry);
           //move backward
           moveBackward(47, true, 5,fast, telemetry);
           //rotate - added since right is not exactly going right
           rotateLeftSide(1,true,5,fast,telemetry);
           //strafe right
           strafeRight(20, true, 5, fast, telemetry);
           //moving backward to starighten robot
           moveBackward(2, true, 5, fast, telemetry);
           //sleeping
           sleep(500);
           //rotate
           rotateLeftSide(1.5,true,5,fast,telemetry);
           //move forward
           moveForward(47, true, 10, slow, telemetry);//48-46-47

           //
           moveBackward(2, true, 5, fast, telemetry);

       }else if(totalRings==1){
           //stopping intake
           runIntake(0);
           /* wobble arm up, turn 180, put wobble arm down, go forward, grip wobble, wobble arm up,
            turn 180, go forward, wobble arm down, release wobble, strafe left, go back.


            */

           //
           //rotateLeft(4.2, true, 5, slow, telemetry);
           //strafe left 25"
           strafeLeft(2,true, 5, fast, telemetry);
           //move backward
           moveBackward(62, true, 5,fast, telemetry);//59
           //moveBackward(4,true, 5,slow,telemetry);
           //rotate - added since right is not exactly going right
           rotateLeftSide(1,true,5,fast,telemetry);
           //strafe right
           strafeRight(27, true, 5, medium, telemetry);//24-26-27
           //moving backward to starighten robot
           //moveBackward(2, true, 5, fast, telemetry);
           rotateLeftSide(1,true,5,fast,telemetry);
           //strafe right
           //strafeRight(12, true, 5, medium, telemetry);//24-26-27
           //moving backward to starighten robot
           moveBackward(2, true, 5, fast, telemetry);
           //rotate
           rotateRightSide(2,true,5,fast,telemetry);//2.2-1.9-1.6-1.7-2
           //move forward
           //moveForward(10, true, 10, medium, telemetry);//64
           //
           moveForward(64, true, 10, medium, telemetry);//64-62-66-64

           moveBackward(2, true, 5, fast, telemetry);
       }
    }

    public void putSecondWobbleUsingArm(int totalrings, Telemetry telemetry){
        if (totalrings == 0){
            runIntake(0);
            strafeLeft(1,true,5,slow, telemetry);
            strafeLeft(11,true,5,fast,telemetry);
            rotateRightSide(35,true,5,fast,telemetry);
            putWobbelArmDown();
            releaseWobble();
            moveForward(13,true,5,fast,telemetry);
            grabWobble();
            moveWobbleArmUp();
            rotateLeftSide(30,true,5,fast,telemetry);
            moveForward(12,true,5,fast,telemetry);
            moveWobbleArmDown();
            releaseWobble();
            strafeLeft(50,true,5,fast,telemetry);
            //strafeLeft(10, true, 5,fast, telemetry);
            //moveForward(5, true, 5, fast, telemetry);
        }else if (totalrings == 1){
              /* wobble arm up, turn 180, put wobble arm down, go forward, grip wobble, wobble arm up,
            turn 18, go forward, wobble arm down, release wobble, strafe left, go back. */
            rotateRightSide(2,true,5,fast,telemetry);
            moveForward(60, true, 10, fast, telemetry);
            moveWobbleArmDown();
            grabWobble();
            rotateLeftSide(2, true, 5, fast, telemetry);
            moveForward(60, true, 10, fast, telemetry);
            releaseWobble();
            putWobbelArmUp();
            //strafeLeft(10, true, 5,fast, telemetry);
            //moveForward(5, true, 5, fast, telemetry);
        }

    }

    public void  shootPowerShots(double lifterPosition){
        // Start shooter motors
        runShooterFront(0.95);
        runShooterBack(0.95);
        // Move the lifter up
        moveLifter(lifterPosition);

    }

   public void pushRingForwardBack(){

        //pusher.setPosition(0);// used to be zero changed on 3-20  it is a forward position
        sleep(500);
        //pusher.setPosition(0.7); // back position is 0.64
   }
   /*************************************************************/

   public void runCarousel(double power){
       carousel.setPower(power);
   }

    public void dumpAndBringbackBox(){
     box.setPosition(0);
     sleep(2000);
     box.setPosition(1);
    }

    public void dumpBox(){
        box.setPosition(0);
    }

   public void bringBoxBack(){
       box.setPosition(1);
   }

   public void moveLiftUp(int targetPosition, double power){
       lift.setTargetPosition(targetPosition);
       lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       lift.setPower(power);
   }
   public void moveLiftDown(int targetPosition, double power){

   }


    /**
     *
     * let's detect rings based on height of the object.
     * @param leftSide
     * @param duckLabel
     * @return int - Level
     */
    public int detectDuckLevel(float leftSide, String duckLabel){
        int level = 1;
        if(LABEL_DUCK.equals(duckLabel)) {
            if (leftSide > 90 && leftSide < 250) {
                level = 2;
            } else if (leftSide > 300) {
                level = 3;
            } else {
                level = 1;
            }
        }
        return level;
    }

     /**
      * This method is to detect the marcker and set level accordingly
      * @param leftSide
      * @param elementLabel
      * @return
      */
     public int detectElementLevel(float leftSide, String elementLabel){
         int level = 1;
         if(LABEL_MARKER.equals(elementLabel)) {
             if (leftSide > 90 && leftSide < 250) {
                 level = 2;
             } else if (leftSide > 300) {
                 level = 3;
             } else {
                 level = 1;
             }
         }
         return level;
     }
    /**
     *
     *
     * @param level
     * @return
     */
    public int getLiftHeight(int level){
        int height = 500;
        if (level == 1){
            height = 600;

        } else if (level == 2){
            height = 1000; //900,1100
        } else if (level == 3){
            height = 1800;
        }
        return height;
    }




}