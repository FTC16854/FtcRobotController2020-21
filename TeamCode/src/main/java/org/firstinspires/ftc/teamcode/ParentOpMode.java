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

@TeleOp(name="Basic: Drive", group="Linear Opmode")
@Disabled
public class ParentOpMode extends LinearOpMode {

    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
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


    ExpansionHubEx expansionHub;    //use for rev extensions

    //Setup Toggles
    Toggle toggleClaw = new Toggle();
    Toggle toggleLift = new Toggle();
    Toggle toggleDirection = new Toggle();

    //Other Global Variables
    //put global variables here...
    double close_claw = .45;
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
        wobbleLift.setDirection(Servo.Direction.FORWARD);
        conveyor.setDirection(CRServo.Direction.FORWARD);

        //Set range for special Servos
        wobbleLift.scaleRange(0.15,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.

        //Set brake or coast modes. Drive motors should match switch on SPARK Mini attached to LF Drive Motor
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            if(emergencyStop()){
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

    public boolean ShooterStartButton(){
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
    public boolean switchsides(){
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

        rotationSpeed = right_sticky_x()*.75;
        robotSpeed = Math.hypot(left_sticky_x(), left_sticky_y());
        movementAngle = Math.atan2(left_sticky_y(), left_sticky_x()) + Math.toRadians(-90); // with 90 degree offset

        if(toggleDirection.toggleButtonDebounced(switchsides())){  //Flip driving direction
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

    public void StopDrive(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    public boolean emergencyStop(){
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


    //More Methods (Functions)

    public void claw() {

        boolean clawClose = toggleClaw.toggleButtonDebounced(clawButton());

        if (clawClose) {
            wobbleClaw.setPosition(close_claw);
            telemetry.addData("Claw:", "Closed");
        } else {
            wobbleClaw.setPosition(open_claw);
            telemetry.addData("Claw:", "Open");
        }
    }

    public void autoclaw(){
        wobbleClaw.setPosition(close_claw);
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
        double intakeServoSpeed = 1;
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

    public void shooter(){
        double shootPosition = .3;  //flipper position
        double neutralPosition = 0;
        //double shooterSpeed = .8;
        double shooterSpeed = 1;

        if(ShooterStartButton()){
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
        sleep(2500);
        shooterFlip();

        //make neutral and shooter positions global variables? incorporate shooterStart and shooterStop into shooter()?
    }

    public void shooterFlip(){
        double shootPosition = .3;
        double neutralPosition = 0;

        shooterFlipper.setPosition(shootPosition);
        sleep(500);
        shooterFlipper.setPosition(neutralPosition);
    }



    //Encoder Functions




    public void shooterTest(){
        double shooterspeed = 0;

        if(gamepad1.a){
            shooterspeed = 0.2;
        }
        else if(gamepad1.x){
            shooterspeed = 0.4;
        }
        else if(gamepad1.y){
            shooterspeed = 0.6;
        }
        else if(gamepad1.b){
            shooterspeed = 0.8;
        }
        else if(gamepad1.right_bumper){
            shooterspeed = 0.9;
        }

        if((shooterspeed > 0) && shootButton()){
            shooterFlip();
            sleep(750);
        }

        telemetry.addData("Shooter Speed",shooterspeed);

        shooterStart(shooterspeed);
    }

    public void testFunction2(){
        //this is a test, too
        return;
    }

    public void testFunction4(){
        //test test test test
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

    public void testFunction1(){
        //this is a test
        return;
    }

}

        //TODO
        //  Odometry/encoders
        //  Gyro
        //      -for drive-straight, auto turning
        //      -field-centric driving
        //  Global Variables
        //      -For shooter flipper positions
        //  correct issues found during robot testing
        //

        //Encoder Stuff
        //Shooter motor - 3.7:1 - 1620 RPM
        //      25.9 Counts per Motor Shaft Revolution
        //      1003.6 Counts per Output Shaft Rotation

        //Odometry Wheels
        //      9192 Counts per revolution


