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

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


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

@Autonomous(name="Ring Thing Odometry", group="Linear Opmode")
//@Disabled
public class AutoParkOnLineOdometryWobbleDropOff extends visionParentOpMode {

    @Override
    public void runOpMode() {

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //select autonomous mode




        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Driveandshoot();
            zeroRings();
            stopDrive();
            saveHeading();
            break;
        }
    }
       public void fourRings(){
            driveInchesVertical(-60,1);
            rotateToHeading(.1,90,'r');
            driveInchesVertical(-12,1);
            autoLiftDown();
            sleep(1000);
            autoClawOpen();
            driveInchesVertical(12,1);
            driveInchesHorizontal(60,1);
           rotateToHeading(.1,0,'l');
       }

        public void oneRings(){
            driveInchesVertical(-48,1);
            autoLiftDown();
            sleep(1000);
            autoClawOpen();
            driveInchesVertical(42,1);
            rotateToHeading(.1,0,'l');
        }

        public void zeroRings(){
            rotateToHeading(.1,90,'r');
            driveInchesHorizontal(14,1);
            driveInchesVertical(18,1);
            autoLiftDown();
            sleep(1000);
            autoClawOpen();
            driveInchesVertical(12,1);
            rotateToHeading(.1,0,'l');
        }
        public void Driveandshoot(){
            autoClawClose();
            autoLiftUp();

            sleep(1000);
            driveInchesVertical(-55, 1);
            driveInchesHorizontal(18, 1);
            for (int i = 0; i < 3; i++) {
                shootAuto(0.45);
            }
            shooterStop();

        }
}



