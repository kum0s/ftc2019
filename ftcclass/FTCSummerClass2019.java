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

package org.firstinspires.ftc.teamcode.ftcclass;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


@Disabled
@TeleOp(name = "FTCSummerClass2019", group = "Teleop")
public class FTCSummerClass2019 extends LinearOpMode {
    //Drive
    DcMotor driveR;
    DcMotor driveL;


    @Override
    public void runOpMode() {
        driveR = hardwareMap.get(DcMotor.class, "driveR");
        driveL = hardwareMap.get(DcMotor.class, "driveL");

        driveR.setDirection(DcMotor.Direction.REVERSE);
        driveL.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {
            //Drive the robot
            handleRobotDrive();

            telemetry.update();
        }
    }


    void handleRobotDrive(){
        //Drive
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double rightPower;
        double leftPower;

        //To edit speeds
        double speedR = 1;
        double speedL = 1;

        double drive = -gamepad1.left_stick_y;
        if(gamepad1.left_bumper) {
            //reduce drive speed when left bumper pressed
            drive = drive/5;
        }
        double turn  =  gamepad1.right_stick_x;
        if(gamepad1.right_bumper){
            //reduce turn speed when right bumper pressed
            turn = turn/4;
        }
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        if(gamepad1.a) {
            // turbo - give all the power
            driveR.setPower(rightPower);
            driveL.setPower(leftPower);
        } else {
            // Send calculated power to wheels
            driveR.setPower(speedR*rightPower);
            driveL.setPower(speedL*leftPower);
            telemetry.addData("right power:", speedR*rightPower);
            telemetry.addData("left  power:", speedL*leftPower);
            telemetry.update();
        }
    }
}