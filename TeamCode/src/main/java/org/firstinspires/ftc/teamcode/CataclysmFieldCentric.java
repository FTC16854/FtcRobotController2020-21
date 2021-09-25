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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name="Cataclysm Field-Centric", group="Linear Opmode")
//@Disabled
public class CataclysmFieldCentric extends LinearOpMode {

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
    private Servo foundationMoverL;
    private Servo foundationMoverR ;
    //set by initialize and runLiftMotot functions.
    //used to determine if it's okay to move armServo
    //which should only happen when in MAX LIFT POSITION
    private double currentLiftPosition;
    private double MAX_LIFT_POSITION;
    private boolean armOut;
    private double armDebounce;

    private BNO055IMU               imu;
    private Orientation lastAngles = new Orientation ();
    private double heading;
    private double headingoffset;

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
        foundationMoverL = hardwareMap.get(Servo.class, "foundationmoverL");
        foundationMoverR = hardwareMap.get (Servo.class, "foundationmoverR");
        foundationMoverL.setDirection (Servo.Direction.REVERSE);


        //begin imu init code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode               = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        //end imu init code

        headingoffset = headingOffsetHolder.getOffset();

        foundationMoverL.setPosition(0);
        foundationMoverR.setPosition(0);
        clawServo.setDirection(Servo.Direction.REVERSE);

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
            heading = getAngle()-headingoffset;
            resetheading();

            moveRobotFieldCentric();
            runIntakeMotor();
            runLiftMotor(gamepad2.left_stick_y);
            foundationMover();
            //runArmServo();
            toggleArmServo();
            runClawServo();
            if(IsEmergency()) {
                break;
            }
            telemetry.addData("lift current pos", currentLiftPosition);
            telemetry.addData("Heading",heading);
            telemetry.update();
        }

        telemetry.addData("Status", "OpMode Stopped.");
        telemetry.update();
    }


    public void moveRobot(){
        double leftStickMovement = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        //
        /*
        if(gamepad1.a){
            leftStickMovement = leftStickMovement*.25;
        }
        */
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double Rotation = -gamepad1.right_stick_x;
        final double lf = leftStickMovement * Math.cos(robotAngle) + Rotation;
        final double rf = leftStickMovement * Math.sin(robotAngle) - Rotation;
        final double lr = leftStickMovement * Math.sin(robotAngle) + Rotation;
        final double rr = leftStickMovement * Math.cos(robotAngle) - Rotation;
        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }
    public void moveRobotFieldCentric(){
        double leftStickMovement = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        leftStickMovement = (0.75 *(leftStickMovement*leftStickMovement)) +0.5*(leftStickMovement);

        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double currentAngle = Math.toRadians(heading);
        robotAngle = robotAngle - currentAngle;

        double Rotation = -gamepad1.right_stick_x;
        if(Rotation < 0) {

            Rotation = - (0.75 *(Rotation*Rotation));
        }
        else {
            Rotation = (0.75 *(Rotation*Rotation));
        }

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
            power = .5; //forward
            runClawServo();
        } else if (gamepad1.left_trigger > 0) {
            power    = -.5; //reverse
            runClawServo();
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
        double minPosition = 15;
        //TODO: Change 10 to max inches for lift to rise - 10 inches was my safe guess
        // double maxPosition = COUNTS_PER_INCH * 26; //should be 10 inches, this will need to be
        //adjusted based on max lift arm height
        double maxPosition = 4010;

        MAX_LIFT_POSITION = maxPosition; //set here so changes to COUNTS_PER_INCH AND INCHES count
        currentLiftPosition = -leftFront.getCurrentPosition(); //remember liftMotor encoder is actually
        // attached to leftFront motor encoder


        if(currentLiftPosition < maxPosition && power > 0){
            liftMotor.setPower(power);

            telemetry.addData("Lift Power:", power);

        } else if(currentLiftPosition > minPosition && power < 0){
            liftMotor.setPower(power);

            telemetry.addData("Lift Power:", power);
        } else if(power == 0 && currentLiftPosition > minPosition + 100) {
            liftMotor.setPower(HOLDING_POWER);
            telemetry.addData("HOLDING AT  Power:", HOLDING_POWER);
        }
        else{

            liftMotor.setPower(0); //fail safe for max and min encoder position

            telemetry.addData("Lift Power:",power);
        }
    }

    /*
        If servo is running in the wrong direction, check the initialize function to setup reverse
     */
    public void runArmServo() {
        //TODO: Change MAX. Currently set to 180 degrees or 180/280
        double MAX = 0.7; //should be 180 degrees, 1.0 should be 280 -- the max for gobuilda servo

        //double liftHeightTolerance = MAX_LIFT_POSITION * .05;
        double liftHeightTolerance = 2120 ;

        if ((currentLiftPosition) > liftHeightTolerance){
            if (gamepad2.left_trigger > 0.1) {
                armServo.setPosition(MAX); //180 degrees - away from robot
                telemetry.addData("leftTrigger",gamepad2.left_trigger);
                telemetry.addData("ServoPosition",armServo.getPosition());
            } else {
                armServo.setPosition(0); //0 degrees - point in towards robot
                telemetry.addData("leftTrigger",gamepad2.left_trigger);
                telemetry.addData("ServoPosition",armServo.getPosition());
            }
        }
    }

    public void toggleArmServo() {
        //TODO: Change MAX. Currently set to 180 degrees or 180/280
        double MAX = 0.7; //should be 180 degrees, 1.0 should be 280 -- the max for gobuilda servo

        //double liftHeightTolerance = MAX_LIFT_POSITION * .05;
        double liftHeightTolerance = 2120 ;
        //  private boolean armOut;
        //private double armDebounce;

        //check time since trigger was last pressed
        if(armDebounce != 0 && armDebounce+500< System.currentTimeMillis()){
            armDebounce=0;
        }

        if(gamepad2.left_bumper && armDebounce==0) {
            armOut =! armOut;
            armDebounce = System.currentTimeMillis();
        }

        if ((currentLiftPosition) > liftHeightTolerance){
            if (armOut == true) {
                armServo.setPosition(MAX); //180 degrees - away from robot
                telemetry.addData("leftTrigger",gamepad2.left_trigger);
                telemetry.addData("ServoPosition",armServo.getPosition());
            } else {
                armServo.setPosition(0); //0 degrees - point in towards robot
                telemetry.addData("leftTrigger",gamepad2.left_trigger);
                telemetry.addData("ServoPosition",armServo.getPosition());
            }
        }
    }

    public void runClawServo() {
        if((gamepad2.right_bumper) || (gamepad1.left_trigger > 0) || (gamepad1.right_trigger > 0)) {
            openClawServo();
        }
        else {
            closeClawServo();
        }
    }

    public void closeClawServo(){
        clawServo.setPosition(0.07);
    }

    public void openClawServo(){
        clawServo.setPosition(0.18); //was .15
    }

    public boolean IsEmergency() {
        boolean emergency = false;

        if((gamepad1.b && gamepad1.y) || (gamepad2.b && gamepad2.y))

        {
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

    public void foundationMover(){
        double downPosition = 0.5;
        double upPosition = 0;
        if (gamepad2.x) {
            foundationMoverL.setPosition(downPosition);
            foundationMoverR.setPosition(downPosition + 0.05);
        }else {
            foundationMoverR.setPosition(upPosition);
            foundationMoverL.setPosition(upPosition);
        }
    }


    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = 0;
    }

    public double getAngle() {
        // Z axis is returned as 0 to +180 or 0 to -180 rolling to -179 or +179 when passing 180
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;
    }
    public void resetheading(){
        if (gamepad1.back){
            sleep(500);
            if (gamepad1.back) {
                headingoffset=0;
                BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
                parameters.mode                = BNO055IMU.SensorMode.IMU;
                parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
                parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
                parameters.loggingEnabled      = false;
                // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
                // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
                // and named "imu".
                imu = hardwareMap.get(BNO055IMU.class, "imu");
                imu.initialize(parameters);

                while (!isStopRequested() && !imu.isGyroCalibrated()){
                    sleep(50);
                    idle();
                }

            }
        }
    }
}
