package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Drive Via Gamepad", group = "ftc16671")
public class MecanumDriveOpMode extends OpMode {
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private double[] distances;
    double pusherPosition, pusherMinPosition, pusherMaxPosition,  lifterPosition, grabberPosition, armPosition;
    double  MIN_POSITION = 0, MAX_POSITION = 1;
    double SERVO_OFFSET = 0.005;
    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        mecanumDrive.initCarousel_and_lift(hardwareMap);
        mecanumDrive.initServo(hardwareMap);
        mecanumDrive.initIntake(hardwareMap);

    }

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //setup gamepads
        //getting gamepad1 controls

        //double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
        double forward = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y*-.6;
        //double strafe = gamepad1.left_stick_x * 1;
        double strafe = Math.abs(gamepad1.left_stick_x)*gamepad1.left_stick_x * .6;
        //double rotate = gamepad1.right_stick_x * 1;
        double rotate = Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x * .6;
        /*
        double gamepadrt = gamepad1.right_trigger;
        double gamepadlt = gamepad1.left_trigger;
        boolean gamepadLeftBumper = gamepad1.left_bumper;
        boolean gamepadRightBumper = gamepad1.right_bumper;
        //getting gamepad 2 controls
        boolean gamepad2X = gamepad2.x;
        boolean gamepad2Y = gamepad2.y;
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        double gamepad2rt = gamepad2.right_trigger;
        double gamepad2lt = gamepad2.left_trigger;
        boolean gamepad2LeftBumper = gamepad2.left_bumper;
        boolean gamepad2RightBumper = gamepad2.right_bumper;

         */
        //Setting gamepad2B***************************


        //Setting gamepad2Y***************************
        if(gamepad2.y){
            if(lifterPosition > 0.2) {
                lifterPosition -= SERVO_OFFSET;
            }
            //mecanumDrive.lifter.setPosition(Range.clip(lifterPosition, MIN_POSITION, MAX_POSITION));
            //telemetry.addData("lifter servo", "position=" + lifterPosition + "  actual="
            //        + mecanumDrive.lifter.getPosition());
        }
        //setting power for carousel and lifter
        /*
        if(gamepad2.b){
            mecanumDrive.carousel.setPower(-1.0);
        }
        if(gamepad2.x){
            mecanumDrive.carousel.setPower(1.0);
        }
        */
        //left trigger is intake forward, right is intake backward
        if (gamepad1.left_trigger > 0){
            //mecanumDrive.intake.setTargetPosition(4000000);
            mecanumDrive.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.runIntake(-gamepad1.left_trigger);//take it in
            telemetry.addData("intake", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        if (gamepad1.right_trigger > 0){
            //mecanumDrive.intake.setTargetPosition(-4000000);
            mecanumDrive.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mecanumDrive.runIntake(gamepad1.right_trigger);//take it in
            telemetry.addData("intake", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        //turn intake power off when gamepad1 b is pressed or right and left trigger both are pressed together
        if (gamepad1.left_trigger <= 0 && gamepad1.right_trigger <= 0){

                mecanumDrive.runIntake(0);//turn off

        }
        // We are making the robot move slower when the bumpers are moved in gamepad1
        if (gamepad1.left_bumper || gamepad1.right_bumper){
            forward = Math.abs(gamepad1.left_stick_y)*gamepad1.left_stick_y*-.2;
            //double strafe = gamepad1.left_stick_x * 1;
            strafe = Math.abs(gamepad1.left_stick_x)*gamepad1.left_stick_x * .2;
            //double rotate = gamepad1.right_stick_x * 1;
            rotate = Math.abs(gamepad1.right_stick_x)*gamepad1.right_stick_x * .2;

        }


        //carousel spins when b is activated
        if (gamepad1.b){
            //mecanumDrive.intake.setTargetPosition(4000000);
            mecanumDrive.carousel.setPower(-0.5);
            telemetry.addData("carousel", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        //carousel spins other way when x is activated
        if (gamepad1.x){
            //mecanumDrive.intake.setTargetPosition(-4000000);
            mecanumDrive.carousel.setPower(-0.5);
            telemetry.addData("carousel", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }

        if(!gamepad1.b && !gamepad1.x){
            mecanumDrive.carousel.setPower(0);
        }


        //left bumper is 0 to 1, right bumper is 1 to 0
        if(gamepad2.left_bumper){
            telemetry.addData("left-before lift", "position=" +  mecanumDrive.intake.getCurrentPosition());
            mecanumDrive.holder.setPosition(.73);
            mecanumDrive.lift.setTargetPosition(500);
            mecanumDrive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanumDrive.lift.setPower(0.8);
            mecanumDrive.box.setPosition(0.9);
            justWait(500);
            mecanumDrive.box.setPosition(0.66);
            telemetry.addData("left-after lift", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        //left bumper is 0 to 1, right bumper is 1 to 0//jsut to commit
        if(gamepad2.right_bumper){
            telemetry.addData("left-before lift", "position=" +  mecanumDrive.intake.getCurrentPosition());
            mecanumDrive.holder.setPosition(.73);
            mecanumDrive.box.setPosition(.97);
            //wait for some milliseconds
            justWait(1050);//1000,750,825,900,950,
            mecanumDrive.lift.setTargetPosition(35);
            mecanumDrive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanumDrive.lift.setPower(0.8);
            telemetry.addData("left-before lift", "position=" +  mecanumDrive.intake.getCurrentPosition());
        }
        //this should go to 500 when position is already up (more than 400, possibly 800,1200 or 1600)
        if(gamepad2.a) {
            if (mecanumDrive.lift.getCurrentPosition() >= 400) {
                telemetry.addData("A position", "=" + mecanumDrive.intake.getCurrentPosition());
                mecanumDrive.lift.setTargetPosition(500);
                mecanumDrive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.lift.setPower(1);

                mecanumDrive.box.setPosition(0.66);
                //telemetry.addData("left-after lift", "position=" + mecanumDrive.intake.getCurrentPosition());
                telemetry.addData("A position", "=" + mecanumDrive.lift.getCurrentPosition());
            }
        }
        //this should go to 900 position b
        if(gamepad2.b) {
            if (mecanumDrive.lift.getCurrentPosition() >= 400) {
                telemetry.addData("B Position", "position=" + mecanumDrive.lift.getCurrentPosition());
                mecanumDrive.lift.setTargetPosition(900);
                mecanumDrive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.lift.setPower(1);

                mecanumDrive.box.setPosition(0.66);
                //telemetry.addData("left-after lift", "position=" + mecanumDrive.intake.getCurrentPosition());
                telemetry.addData("B Position", "position=" + mecanumDrive.lift.getCurrentPosition());
            }
        }
        //this should go to 1600 position x
        if(gamepad2.x) {
            if (mecanumDrive.lift.getCurrentPosition() >= 400) {
            telemetry.addData("X Position", "position=" + mecanumDrive.lift.getCurrentPosition());
            mecanumDrive.lift.setTargetPosition(1800);//1600, 1750,1800
            mecanumDrive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mecanumDrive.lift.setPower(1);

            mecanumDrive.box.setPosition(0.66);
            //telemetry.addData("left-after lift", "position=" + mecanumDrive.intake.getCurrentPosition());
            telemetry.addData("X Position", "position=" + mecanumDrive.lift.getCurrentPosition());
            }
        }
        //this should go to 1800 position y
        if(gamepad2.y) {
            if (mecanumDrive.lift.getCurrentPosition() >= 400) {
                telemetry.addData("Y Position", "position=" + mecanumDrive.lift.getCurrentPosition());
                mecanumDrive.lift.setTargetPosition(2200);
                mecanumDrive.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mecanumDrive.lift.setPower(1);

                mecanumDrive.box.setPosition(0.66);
                //telemetry.addData("left-after lift", "position=" + mecanumDrive.intake.getCurrentPosition());
                telemetry.addData("Y Position", "position=" + mecanumDrive.lift.getCurrentPosition());
            }
        }

        //Dump function - press doad-down to dump and return
        if (mecanumDrive.lift.getCurrentPosition() >= 450) {
            if (gamepad2.dpad_down) {
                mecanumDrive.box.setPosition(0);
                //wait here for some milliseconds
                justWait(1500);
                mecanumDrive.box.setPosition(0.66);
                mecanumDrive.lift.setTargetPosition(500);
                mecanumDrive.holder.setPosition(.73);

            }
            //Dump function - press doad down to dump (and not return - for capstone aiming)
            if (gamepad2.dpad_up) {
                mecanumDrive.box.setPosition(0);
            }
            //telemetry.addData("left-after lift", "position=" + mecanumDrive.intake.getCurrentPosition());
            telemetry.addData("box Position", "position=" + mecanumDrive.lift.getCurrentPosition());
        }
        //Hold the Capstone function - press doad right to HOLD the capstone
        if (mecanumDrive.lift.getCurrentPosition() >= 300) {
            if (gamepad2.dpad_right) {
                mecanumDrive.holder.setPosition(.8);
            }
        }
        //Hold the Capstone function - press doad left to RELEASE the capstone
        if (mecanumDrive.lift.getCurrentPosition() >= 300) {
            if (gamepad2.dpad_left) {
                mecanumDrive.holder.setPosition(.73);
            }
        }


        //carousel is right joystick up and down, lift is left joystick up and down
        //mecanumDrive.carousel.setPower(gamepad2.right_stick_y);

        //mecanumDrive.lift.setPower(gamepad2.left_stick_y);

        //supply gamepad values to run motors, servo and other parts of robots
        mecanumDrive.driveMecanum(forward, strafe, rotate);
        distances = mecanumDrive.getDistanceCm();
        telemetry.addData("distance fwd", distances[0]);
        telemetry.addData("distance right", distances[1]);
        telemetry.update();
    }

    /**
     *
     * @param miliseconds
     */

    private void justWait(int miliseconds) {

        double currTime = getRuntime();
        double waitUntil = currTime + (double)(miliseconds/1000);
        while (getRuntime() < waitUntil){
            //do nothing
        }

    }
}