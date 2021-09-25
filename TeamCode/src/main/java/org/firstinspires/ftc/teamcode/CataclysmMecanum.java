/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Cataclysm Mecanum", group="Linear Opmode")
//@Disabled
public class CataclysmMecanum extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;
    private DcMotorSimple intakeMotors = null;
    private DcMotorSimple liftMotor = null;
    private Servo armServo = null;
    private Servo clawServo = null;

    //set by initialize and runLiftMotot functions.
    //used to determine if it's okay to move armServo
    //which should only happen when in MAX LIFT POSITION
    private double currentLiftPosition;
    private double MAX_LIFT_POSITION;


    /*
        DC and Servo motor setup, this method should be called first in the opmode method.
        You can change motor and servo direction in by removing comments in this code
     */
    public void Initialize() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "FL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        leftRear  = hardwareMap.get(DcMotor.class, "BL");
        rightRear = hardwareMap.get(DcMotor.class, "BR");
        intakeMotors = hardwareMap.get(DcMotorSimple.class, "im");
        liftMotor = hardwareMap.get(DcMotorSimple.class, "lm");
        armServo = hardwareMap.get(Servo.class, "arm");
        clawServo = hardwareMap.get(Servo.class,"claw");

        // clawServo.setDirection(Servo.Direction.REVERSE);

        //TODO: setup armServo, it it's moving backwards, uncomment next line
        //armServo.setDirection(Servo.Direction.REVERSE);
        //setup arm servo max travel


        //TODO: Possibly uncomment if lift motor is running backwards
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //TODO: ready the leftFront motor encoder, that we're using for liftMotor
        //make sure position is 0 when the lift arm is all the way down
        //which should be done manually if it's left upright when the robot is off
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //run without means user power instead of speed, not that encoders don't count
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentLiftPosition =  leftFront.getCurrentPosition(); //should be 0 based on stop and reset
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //setup dc and servo motors
        Initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            moveRobot();
            runIntakeMotor();


            runLiftMotor(gamepad2.left_stick_y);

            /*
            if(gamepad2.a){
                runLiftMotor(1);
            }
            else if(gamepad2.b) {
                runLiftMotor(-1);
            }
            */



            runArmServo();
            runClawServo();
            if(IsEmergency()) {
                break;
            }
            telemetry.addData("lift current pos", currentLiftPosition);
            telemetry.update();
        }
        telemetry.addData("Status", "OpMode Stopped.");
        telemetry.update();
    }


    public void moveRobot(){
        double leftStickMovement = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double Rotation = gamepad1.right_stick_x;
        final double lf = leftStickMovement * Math.cos(robotAngle) + Rotation;
        final double rf = leftStickMovement * Math.sin(robotAngle) - Rotation;
        final double lr = leftStickMovement * Math.sin(robotAngle) + Rotation;
        final double rr = leftStickMovement * Math.cos(robotAngle) - Rotation;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }


    public void runIntakeMotor(){
        double power = 0.0;
        if (gamepad1.right_trigger > 0) {
            power = 1.0; //forward
            runClawServo();
        } else if (gamepad1.left_trigger > 0) {
            power    = -1.0; //reverse

        } else if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            power = 0.0; //stop
        } else {
            power = 0.0; //catchall to stop intake if no conditions are matched
        }
        intakeMotors.setPower(power);
    }


    public void runLiftMotor(double power){
        power = -power;
        double HOLDING_POWER = 0.02;

        double COUNTS_PER_MOTOR_REV    = 720.0 ;    // eg: gobuilda Motor Encoder
        double DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
        //TODO: Measure lift motor wheel diameter (in inches) and change 2.0 to that
        double WHEEL_DIAMETER_INCHES   = 1.9740 ;     // For figuring circumference,
        // may need to be changed for lift motor
        double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415); // 114.5949 counts per inch for lift motor
        // guessing it was about 2 inches from memory

        //TODO: Assuming lift motor down is leftMotor encoder 0 since we reset on initialize
        double minPosition = 0;
        //TODO: Change 10 to max inches for lift to rise - 10 inches was my safe guess
        double maxPosition = COUNTS_PER_INCH * 20; //should be 10 inches, this will need to be
        //adjusted based on max lift arm height

        MAX_LIFT_POSITION = maxPosition; //set here so changes to COUNTS_PER_INCH AND INCHES count
        currentLiftPosition = -leftFront.getCurrentPosition(); //remember liftMotor encoder is actually
        // attached to leftFront motor encoder


        if(currentLiftPosition < maxPosition && power > 0){
            liftMotor.setPower(power);

            telemetry.addData("Power:", power);

        } else if(currentLiftPosition > minPosition && power < 0){
            liftMotor.setPower(power);

            telemetry.addData("Power:", power);
        }
        else{
            if(currentLiftPosition > 5){
                liftMotor.setPower(HOLDING_POWER);
            }
            //    liftMotor.setPower(0); //fail safe for max and min encoder position

            telemetry.addData("Power:",power);
        }

        //Should be cleaned up and/or removed. Doesn't seem necessary based on above logic
        //but it works to prevent travel in negative direction

        if(currentLiftPosition < 0){
            if(currentLiftPosition < maxPosition && power > 0) {
                liftMotor.setPower(power);

                telemetry.addData("Power:", power);
            }
            else{
                liftMotor.setPower(0);
            }
        }



    }

    /*
        If servo is running in the wrong direction, check the initialize function to setup reverse
     */
    public void runArmServo() {
        //TODO: Change MAX. Currently set to 180 degrees or 180/280
        double MAX = 0.64; //should be 180 degrees, 1.0 should be 280 -- the max for gobuilda servo

        //Make sure the lift is up before allowing arm to swing in or out

        //if(currentLiftPosition > (MAX_LIFT_POSITION - liftHeightTolerance)) {
        if(currentLiftPosition > (MAX_LIFT_POSITION)) {

            if (gamepad2.left_trigger > 0) {
                armServo.setPosition(MAX); //180 degrees - away from robot
            } else {
                armServo.setPosition(0); //0 degrees - point in towards robot
            }
        }
    }

    public void runClawServo() {
        //TODO: Change MAX. Currently set to 5 degrees or 5/280
        //   double MAX = 0.0178; //guess claw grip at 5 degree servo movement, may need to adjust
        double MAX = 0.2; //guess claw grip at 5degree servo movement, may need to adjust

        if(gamepad2.right_trigger > 0) {
            clawServo.setPosition(MAX);
        }
        else {
            clawServo.setPosition(0);
        }
    }


    public boolean IsEmergency() {
        boolean emergency = false;

        if(gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down
                || gamepad1.dpad_up || gamepad1.b || gamepad2.dpad_up || gamepad2.dpad_down
                || gamepad2.dpad_left || gamepad2.dpad_right ) {
            emergency = true;
            //power down all motors
            //servos should stop when breaking out of opmode active loop
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
            liftMotor.setPower(0);
        }

        return emergency;
    }
}