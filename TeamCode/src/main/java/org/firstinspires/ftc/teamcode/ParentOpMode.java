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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;

import org.openftc.revextensions2.ExpansionHubEx;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Parent Opmode", group="Linear Opmode")
@Disabled
public class ParentOpMode extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();    // Declare OpMode members.

    private DcMotorSimple leftFront = null; //DcMotorSimple because it is connected to SPARK Mini
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotorEx shooterMotor = null;   //DcMotorEx offers extended capabilities such as setVelocity()
    private CRServo intakeServo = null;
    private Servo shooterFlipper = null;
    //private Servo intakeLatch = null;     //Intake Latch is hopefully not needed. No more ports :(
    private Servo wobbleClaw = null;
    private ServoImplEx wobbleLift = null; //ServoImplEx for PWM range adjustment
    private CRServo conveyor  = null;
    // The IMU sensor object JWN Pulled from SensorBNO055IMU Opmode
    BNO055IMU imu;
    // State used for updating telemetry JWN Pulled from SensorBNO055IMU Opmode
    Orientation angles = new Orientation();
    double heading;
    double headingOffset;

    ExpansionHubEx expansionHub;    //use for rev extensions

    //Setup Toggles
    Toggle toggleClaw = new Toggle();
    Toggle toggleLift = new Toggle();
    Toggle toggleDirection = new Toggle();
    Toggle toggleShootermotor = new Toggle();

    //Other Global Variables
    //put global variables here...
    double close_claw = .45;    //claw positions
    double open_claw = 0;

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotorSimple.class, "lf_drive");
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        leftBack  = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");
        shooterFlipper = hardwareMap.get(Servo.class,"shooterFlipper_servo");
        //intakeLatch = hardwareMap.get(Servo.class,"intakeLatch_servo");
        wobbleClaw = hardwareMap.get(Servo.class, "wobble_claw");
        wobbleLift = hardwareMap.get(ServoImplEx.class, "wobble_lift");
        conveyor = hardwareMap.get(CRServo.class, "conveyor_servo");


        //Set motor run mode
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Motor  and servo Directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor.setDirection(DcMotorEx.Direction.FORWARD);

        intakeServo.setDirection(CRServo.Direction.REVERSE);
        shooterFlipper.setDirection(Servo.Direction.REVERSE);
        //intakeLatch.setDirection(Servo.Direction.FORWARD);
        wobbleClaw.setDirection(Servo.Direction.FORWARD);
        wobbleLift.setDirection(Servo.Direction.REVERSE);
        conveyor.setDirection(CRServo.Direction.FORWARD);

        //Set range for special Servos
       // wobbleLift.scaleRange(0.15,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.
        wobbleLift.scaleRange(0.20,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.

        //Set brake or coast modes. Drive motors should match switch on SPARK Mini attached to LF Drive Motor
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // JWN Referenced https://stemrobotics.cs.pdx.edu/node/7265 for IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;  //this was the way we did it last year, my be slow and jerky
       // parameters.mode                = BNO055IMU.SensorMode.GYRO; //This should theoretically be faster (no wasted hardware cycles to get pitch and roll) if it works. Haven’t tested yet.
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. IMU to be attached to an I2C port
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        autoClawClose();
        autoLiftUp();

        getHeadingOffsetReal();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    public void runOpMode() {

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //releaseLatch();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            tankdrive();
            intake();
            shooter();
            if(emergencyStopped()){
                break;
            }

            telemetry.update();
        }
    }


    //CONTROLLER MAP
    //Thumbsticks
    public double left_sticky_x(){
        return gamepad1.left_stick_x;
    }

    public double left_sticky_y(){
        return -gamepad1.left_stick_y;
    }

    public double right_sticky_x() {
        return gamepad1.right_stick_x;
    }

    public double right_sticky_y() {
        return -gamepad1.right_stick_y;
    }

    //Buttons
    public boolean emergencyButtons(){
        if(((gamepad1.y)&&(gamepad1.b))||((gamepad2.y)&&(gamepad2.b))){
            return true; }
        else {
            return false;
        }
    }

    public boolean clawButton(){
        return gamepad1.x;
    }

    public boolean liftButton(){
        return gamepad1.a;
    }

    public boolean shootButton(){
        if((gamepad1.right_trigger>.25)||(gamepad2.right_trigger>.25)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean shooterStartButton(){
        if(gamepad1.right_bumper||gamepad2.right_bumper){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean intakeButton(){
        if((gamepad1.left_trigger>.25)||(gamepad2.left_trigger>.25)){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean outtakeButton(){
        if(gamepad1.left_bumper||gamepad2.left_bumper){
            return true;
        }
        else{
            return false;
        }
    }

    public boolean switchSidesButton(){
        if(gamepad1.b ||gamepad2.b) {
            return true;
        }
        else{
            return false;
        }
    }


    //Drive Methods

    public void tankdrive(){
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        leftPower = left_sticky_y();
        rightPower = right_sticky_y();

        // Send calculated power to wheels
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        // Show the elapsed game time.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    public void holonomicDrive(){
        double robotSpeed;
        double movementAngle;
        double rotationSpeed;

        rotationSpeed = right_sticky_x()*.5;
        robotSpeed = Math.hypot(left_sticky_x(), left_sticky_y());
        movementAngle = Math.atan2(left_sticky_y(), left_sticky_x()) + Math.toRadians(-90); // with 90 degree offset

        if(toggleDirection.toggleButtonDebounced(switchSidesButton())){  //Flip driving direction
            movementAngle = movementAngle + Math.toRadians(180);
        }
        double leftFrontSpeed = (robotSpeed * Math.cos(movementAngle + (Math.PI / 4))) + rotationSpeed;
        double rightFrontSpeed = (robotSpeed * Math.sin(movementAngle + (Math.PI / 4))) - rotationSpeed;
        double leftBackSpeed = (robotSpeed*Math.sin(movementAngle + (Math.PI/4))) + rotationSpeed;
        double rightBackSpeed = (robotSpeed*Math.cos(movementAngle + (Math.PI/4))) - rotationSpeed;

        leftFront.setPower(leftFrontSpeed);
        rightFront.setPower(rightFrontSpeed);
        leftBack.setPower(leftBackSpeed);
        rightBack.setPower(rightBackSpeed);

        telemetry.addData("LF Speed:",leftFrontSpeed);
        telemetry.addData("LB Speed:",leftBackSpeed);
        telemetry.addData("RF Speed:",rightFrontSpeed);
        telemetry.addData("RB Speed:",rightBackSpeed);
    }

    public void fieldCentricDrive(){
        double robotSpeed;
        double movementAngle;
        double rotationSpeed;
        double currentHeading;
        double movementFieldAngle;

        rotationSpeed = right_sticky_x()*.5;
        robotSpeed = Math.hypot(left_sticky_x(), left_sticky_y());
        movementAngle = Math.atan2(left_sticky_y(), left_sticky_x()) - Math.toRadians(180); // with corrected degree offset
        currentHeading = Math.toRadians(heading);
        movementFieldAngle = (movementAngle - currentHeading);

        double leftFrontSpeed = (robotSpeed * Math.cos(movementFieldAngle + (Math.PI / 4))) + rotationSpeed;
        double rightFrontSpeed = (robotSpeed * Math.sin(movementFieldAngle + (Math.PI / 4))) - rotationSpeed;
        double leftBackSpeed = (robotSpeed*Math.sin(movementFieldAngle + (Math.PI/4))) + rotationSpeed;
        double rightBackSpeed = (robotSpeed*Math.cos(movementFieldAngle + (Math.PI/4))) - rotationSpeed;

        leftFront.setPower(leftFrontSpeed);
        rightFront.setPower(rightFrontSpeed);
        leftBack.setPower(leftBackSpeed);
        rightBack.setPower(rightBackSpeed);

        telemetry.addData("LF Speed:",leftFrontSpeed);
        telemetry.addData("LB Speed:",leftBackSpeed);
        telemetry.addData("RF Speed:",rightFrontSpeed);
        telemetry.addData("RB Speed:",rightBackSpeed);
    }

    public void holonomicDriveAuto(double robotSpeed, double movementAngle, double rotationSpeed){

        movementAngle = Math.toRadians (movementAngle - (90));

        double leftFrontSpeed = robotSpeed*Math.cos(movementAngle + (Math.PI/4)) + rotationSpeed;
        double rightFrontSpeed = robotSpeed*Math.sin(movementAngle + (Math.PI/4)) - rotationSpeed;
        double leftBackSpeed = robotSpeed*Math.sin(movementAngle + (Math.PI/4)) + rotationSpeed;
        double rightBackSpeed = robotSpeed*Math.cos(movementAngle + (Math.PI/4)) - rotationSpeed;

        leftFront.setPower(leftFrontSpeed);
        rightFront.setPower(rightFrontSpeed);
        leftBack.setPower(leftBackSpeed);
        rightBack.setPower(rightBackSpeed);
    }

    public void stopDrive(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public boolean emergencyStopped(){
        if (emergencyButtons()) {
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            intakeServo.setPower(0);
            shooterMotor.setPower(0);
            return true;
        }
        else {
            return false;
        }
    }

    //imu functions
    public double getAngle() {
        // Z axis is returned as 0 to +180 or 0 to -180 rolling to -179 or +179 when passing 180
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {  //JWN Added for IMU
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){  //JWN Added for IMU
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void updateHeading() {  // Run during every cycle of teliop
        heading = getAngle()-headingOffset;
        resetheading();
    }

    public void saveHeading() {  //Used after running autonomous to save the current heading
        heading = getAngle();
        headingOffsetHolder.setOffset(heading);
    }


    public void resetheading(){  //Used during teliop if the robot heading needs to be reset
        if (gamepad1.back){
            sleep(500);
            if (gamepad1.back) {
                headingOffset=0;
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

    public void getHeadingOffsetReal () { //we used the name getHeadingOffsetReal because getHeadingOffset was already taken and this one is the real one as shown with the "real" - Cameron
        headingOffset = headingOffsetHolder.getOffset();
    }


    //More Methods (Functions)

    public void claw() {

        boolean clawClose = toggleClaw.toggleButtonDebounced(clawButton());

        if (!clawClose) {
            wobbleClaw.setPosition(close_claw);
            telemetry.addData("Claw:", "Closed");
        } else {
            wobbleClaw.setPosition(open_claw);
            telemetry.addData("Claw:", "Open");
        }
    }

    public void autoClawClose(){
        wobbleClaw.setPosition(close_claw);
    }

    public void autoClawOpen() {
        wobbleClaw.setPosition(open_claw);
    }

    public void autoLiftUp(){
        double up = 1;
        wobbleLift.setPosition(up);
        telemetry.addData("Wobble Lift:","Up");
    }

    public void autoLiftDown() {
        double down = 0;
        wobbleLift.setPosition(down);
        telemetry.addData("Wobble Lift:", "Down");
    }

    public void lift() {
        double down = 0;
        double up = 1;
        boolean liftDown = toggleLift.toggleButtonDebounced(liftButton());

        if (liftDown) {
            wobbleLift.setPosition(down);
            telemetry.addData("Wobble Lift:","Down");
        } else {
            wobbleLift.setPosition(up);
            telemetry.addData("Wobble Lift:","Up");
        }
    }


    public void intake(){
        double intakeServoSpeed = .7;
        double conveyorServoSpeed = 1;

        if(intakeButton()){
            intakeServo.setPower(intakeServoSpeed);
            conveyor.setPower(conveyorServoSpeed);
            telemetry.addData("Intake:","IN");
        }
        else{
            if (outtakeButton()){
            intakeServo.setPower(-intakeServoSpeed);
            conveyor.setPower(-conveyorServoSpeed);
                telemetry.addData("Intake:","OUT");
            }
            else{
                intakeServoSpeed = 0;
                conveyorServoSpeed = 0;
                intakeServo.setPower(intakeServoSpeed);
                conveyor.setPower(conveyorServoSpeed);
                telemetry.addData("Intake:","Stopped");
            }
        }
    }

    //Shooter Functions
    public void shooter(){
        double shootPosition = .3;  //flipper position
        double neutralPosition = 0;
        //double shooterSpeed = .8;
        double shooterSpeed = .5;

        if(toggleShootermotor.toggleButtonDebounced(shooterStartButton())){
            shooterMotor.setPower(shooterSpeed);
            telemetry.addData("Shooter:","Spinning");
            if(shootButton()){
                shooterFlipper.setPosition(shootPosition);
                telemetry.addData("Shooter:","FIRE");
            }
           else {
               shooterFlipper.setPosition(neutralPosition);
            }
        }
        else{
            shooterFlipper.setPosition(neutralPosition);
            shooterMotor.setPower(0);
            telemetry.addData("Shooter:","Stopped");
        }
    }

    public void shooterStart(double speed){
        shooterMotor.setPower(speed);
    }

    public void shooterStop(){
        shooterMotor.setPower(0);
    }

    public void shootAuto(double speed){

        shooterStart(speed);
        sleep(3000);
        shooterFlip();

        //make neutral and shooter positions global variables? incorporate shooterStart and shooterStop into shooter()?
    }

    public void shooterFlip(){
        double shootPosition = .3;
        double neutralPosition = 0;

        shooterFlipper.setPosition(shootPosition);
        sleep(250);
        shooterFlipper.setPosition(neutralPosition);
        sleep ( 250);
    }



    //Encoder Functions
    public double getLeftVerticalEncoder(){
        return -rightFront.getCurrentPosition();
    }

    public double getRightVerticalEncoder(){
        return leftBack.getCurrentPosition();
    }

    public double getHorizontalEncoder(){
        return rightBack.getCurrentPosition();
    }

    public void driveInchesHorizontal(double distanceInches,double speed) {
        double OdometryWheelDiameter = 3;
        double odometryCircumfrence = Math.PI * OdometryWheelDiameter;
        double countsPerRotation = 8192;
        double targetrotations = distanceInches / odometryCircumfrence;
        double countsToTravel = targetrotations * countsPerRotation;
        double targetCount = countsToTravel + getHorizontalEncoder();


        if (distanceInches > 0) {


            while ((getHorizontalEncoder() < targetCount)&& opModeIsActive()) {
                //MOVE LEFT: movement angleAngle of 359 veers right, 1 veers left
                holonomicDriveAuto(speed, 0, 0);
                telemetry.addData("target", targetCount);
                telemetry.addData("current possision", getHorizontalEncoder());
                telemetry.update();

                stopDrive();
            }
        } else {
            while(getHorizontalEncoder() > targetCount) {
                //MOVE RIGHT: movement angleAngle of 181 should veer left, 179 should veer right
                holonomicDriveAuto(speed, 180, 0);
                telemetry.addData("target", targetCount);
                telemetry.addData("current possision", getHorizontalEncoder());
                telemetry.update();

                stopDrive();
            }
        }
    }

        public void driveInchesVertical(double distanceInches,double speed){
            double OdometryWheelDiameter = 3;
            double odometryCircumfrence = Math.PI * OdometryWheelDiameter;
            double countsPerRotation=8192;
            double targetrotations = distanceInches/odometryCircumfrence;
            double countsToTravel = targetrotations*countsPerRotation;
            double targetCount = countsToTravel + getLeftVerticalEncoder();


            if(distanceInches > 0) {


                while (getLeftVerticalEncoder() < targetCount) {
                    //MOVE shooter end Backwards: movement angleAngle of 91 goes to left, -89 goes right
                    holonomicDriveAuto(speed, 90, 0);
                    telemetry.addData("target", targetCount);
                    telemetry.addData("current possision", getLeftVerticalEncoder());
                    telemetry.update();

                    stopDrive();
                }
            }
            else {
                while (getLeftVerticalEncoder() > targetCount) {
                    //MOVE shooter end Forwards: movement angleAngle of -91 goes to right, -89 goes left
                    holonomicDriveAuto(speed, -90, 0);
                    telemetry.addData("target", targetCount);
                    telemetry.addData("current possision", getLeftVerticalEncoder());
                    telemetry.update();

                    stopDrive();
                }
            }
            }


//Autonomous Rotation Code
public void rotateToHeading(double turnSpeed, double desiredHeading, char rl){
        //NOTE: This code cannot rotate to / through/ across the 180* point
        heading = getAngle();

         if(rl == 'l'){
             while (heading < desiredHeading) {
                 holonomicDriveAuto(0,0,-turnSpeed);
                 heading = getAngle();
                 telemetry.addData("heading:",heading);
                 telemetry.update();

             }
         }
         else{
             while ( heading > desiredHeading) {
                 holonomicDriveAuto(0,0,turnSpeed);
                 heading = getAngle();
                 telemetry.addData("heading:",heading);
                 telemetry.update();
             }
         }
         stopDrive();

}

    public void shooterTest(){
        double shooterspeed = 0;

        if(gamepad1.a){
            shooterspeed = 0.445;
        }
        else if(gamepad1.x){
            shooterspeed = 0.475;
        }
        else if(gamepad1.y){
            shooterspeed = 0.5;
        }
        else if(gamepad1.b){
            shooterspeed = 0.525;
        }
        else if(gamepad1.right_bumper){
            shooterspeed = 0.555;
        }

        if((shooterspeed > 0) && shootButton()){
            shooterFlip();
            sleep(750);
        }

        telemetry.addData("Shooter Speed",shooterspeed);

        shooterStart(shooterspeed);
    }


/*
    public void releaseLatch(){
        double releaseLatchPosition = .5;

        intakeLatch.setPosition(releaseLatchPosition);

    }
*/

    public void getCurrentTelemetry(){
        try{
            double totalCurrent = expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            double motorCurrent0 = expansionHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS,0);
            double motorCurrent1 = expansionHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS,1);
            double motorCurrent2 = expansionHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS,2);
            double motorCurrent3 = expansionHub.getMotorCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS,3);
            double motorCurrentTotal = motorCurrent0 + motorCurrent1 + motorCurrent2 + motorCurrent3;

            double ioCurrent = expansionHub.getGpioBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            double i2cCurrent = expansionHub.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            double servoCurrent = totalCurrent-(motorCurrentTotal+ioCurrent+i2cCurrent); //calculate servo current (bug prevents getting current directly)

            telemetry.addData("Total Current", "totalCurrent");
            telemetry.addData("Motor Current", "motorCurrent");
            telemetry.addData("Servo Current", "servoCurrent");
            }
        catch(Exception currentERROR){
            telemetry.addData("Current Monitoring:", "N/A");
            telemetry.addData("Current Monitoring:", currentERROR);
        }
    }

    //Matt's Test Code
    /*
    public void driveInchesVertical(double distanceInches,double speed){    //negative distance to move back
        double odometryWheelDiameter = 3;
        double odometryWheelCircumfrence = Math.PI * odometryWheelDiameter;
        double countsPerRotation=9192;
        double targetRotations = distanceInches/odometryWheelCircumfrence;
        double countsToTravel = targetRotations*countsPerRotation;

        double targetCounts = countsToTravel + getLeftVerticalEncoder(); //get target position (counts) by adding
        //counts to travel to current position

        if(distanceInches < 0){
            while(getLeftVerticalEncoder() > targetCounts){
                holonomicDriveAuto(speed,-90,0);
                telemetry.addData("Target Counts:",targetCounts);
                telemetry.addData("Current Position:",getLeftVerticalEncoder());
                telemetry.update();
            }
        }
        else{
            while(getLeftVerticalEncoder() < targetCounts){
                holonomicDriveAuto(speed,90,0);
                telemetry.addData("Target Counts:",targetCounts);
                telemetry.addData("Current Position:",getLeftVerticalEncoder());
                telemetry.update();
            }
        }

        stopDrive();

    }

    public void driveInchesHorizontal(double distanceInches,double speed){ //negative distance to move left
        double odometryWheelDiameter = 3;
        double odometryWheelCircumfrence = Math.PI * odometryWheelDiameter;
        double countsPerRotation=9192;
        double targetRotations = distanceInches/odometryWheelCircumfrence;
        double countsToTravel = targetRotations*countsPerRotation;

        double targetCounts = countsToTravel + getHorizontalEncoder();

        if(distanceInches < 0){
            while(getHorizontalEncoder() > targetCounts){
                holonomicDriveAuto(speed,180,0);
                telemetry.addData("Target Counts:",targetCounts);
                telemetry.addData("Current Position:",getHorizontalEncoder());
            }
        }
        else{
            while(getHorizontalEncoder() < targetCounts){
                holonomicDriveAuto(speed,0,0);
                telemetry.addData("Target Counts:",targetCounts);
                telemetry.addData("Current Position:",getHorizontalEncoder());
            }
        }

        stopDrive();

    }
*/


}

        //TODO
        //  Odometry/encoders
        //  Gyro
        //      -for drive-straight, auto turning
        //  Global Variables
        //      -For shooter flipper positions
        //
        //

        //Encoder Stuff
        //Shooter motor - 3.7:1 - 1620 RPM
        //      25.9 Counts per Motor Shaft Revolution
        //      1003.6 Counts per Output Shaft Rotation

        //Odometry Wheels
        //      8192 Counts per revolution


