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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "", group = "Auto")
@Disabled
public abstract class RobotDrive extends RobotInitialization {
    //----------------------------------------------------------------------------------------------
    //
    ///////// driver controller mecanum drive used druing driver controller period /////////////////
    //
    //
    // gamepad1.left_stick_y - back and forth drive
    // gamepad1.left_stick_x - strafe
    // gamepad1.right_stick_x - turn
    // gamepad1.right_bumper or gamepad1.left_bumper - reduce speed to 0.3
    //----------------------------------------------------------------------------------------------
    void mecanumDrive() {
        double fwdBackPower, strafePower, turnPower, maxPower;
        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;


        fwdBackPower= -gamepad1.left_stick_y;

        if (-0.25 < gamepad1.left_stick_x && gamepad1.left_stick_x < 0.25) {
            strafePower = 0;
        } else {
            strafePower= gamepad1.left_stick_x;
        }

        turnPower= gamepad1.right_stick_x;

        leftFrontPower= fwdBackPower + turnPower + strafePower;
        rightFrontPower= fwdBackPower - turnPower - strafePower;
        leftBackPower= fwdBackPower + turnPower - strafePower;
        rightBackPower= fwdBackPower - turnPower + strafePower;

        maxPower = Math.abs(leftFrontPower);
        if(Math.abs(rightFrontPower) > maxPower) {maxPower = Math.abs(rightFrontPower);}
        if(Math.abs(leftBackPower) > maxPower) {maxPower = Math.abs(leftBackPower);}
        if(Math.abs(rightBackPower) > maxPower) {maxPower = Math.abs(rightBackPower);}

        if (maxPower > 1) {
            leftFrontPower = leftFrontPower/maxPower;
            rightFrontPower = rightFrontPower/maxPower;
            leftBackPower = leftBackPower/maxPower;
            rightBackPower = rightBackPower/maxPower;
        }

        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            leftFrontPower = leftFrontPower * 0.3;
            rightFrontPower = rightFrontPower * 0.3;
            leftBackPower = leftBackPower * 0.3;
            rightBackPower = rightBackPower * 0.3;
        }
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);

        telemetry.update();
    }
    //----------------------------------------------------------------------------------------------
    //
    //////// forward and backward drive for both normal wheels and mecanum wheels  /////////////////
    //
    //----------------------------------------------------------------------------------------------
    void move(double motorPower, long time) {
        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(motorPower);
        leftBackMotor.setPower(motorPower);
        rightBackMotor.setPower(motorPower);
        sleep(time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void moveUsingEncoder(int motorPos, double motorPower){
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(motorPower);
        leftBackMotor.setPower(motorPower);
        rightBackMotor.setPower(motorPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            idle();
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void moveUsingEncoder(int motorPos, double motorPower, float angleOne){
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angleTwo = angles.firstAngle;
        double correction = angleTwo - angleOne;

        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(motorPower);
        leftBackMotor.setPower(motorPower);
        rightBackMotor.setPower(motorPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleTwo = angles.firstAngle;
            correction = angleTwo - angleOne;
            correction *= 0.1;
            telemetry.addData("Correction", correction);

            leftFrontMotor.setPower(motorPower - correction);
            rightFrontMotor.setPower(motorPower + correction);
            leftBackMotor.setPower(motorPower - correction);
            rightBackMotor.setPower(motorPower + correction);

            telemetry.addData("Angle", angleTwo);
            telemetry.update();
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    //----------------------------------------------------------------------------------------------
    //
    //////////////////////// strafe (sideways drive) only mecanum wheels  //////////////////////////
    //
    //----------------------------------------------------------------------------------------------
    void strafe(double strafe, long time) {
        leftFrontMotor.setPower(strafe);
        rightFrontMotor.setPower(-strafe);
        leftBackMotor.setPower(-strafe);
        rightBackMotor.setPower(strafe);
        sleep(time);

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void encoderStrafe(int motorPos, double motorPower){
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(-motorPower);
        leftBackMotor.setPower(-motorPower);
        rightBackMotor.setPower(motorPower);
        //sleep(time);

        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            idle();
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void encoderStrafe(int motorPos, double motorPower, float angleOne){
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angleTwo = angles.firstAngle;
        double correction = angleTwo - angleOne;

        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(-motorPower);
        leftBackMotor.setPower(-motorPower);
        rightBackMotor.setPower(motorPower);


        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleTwo = angles.firstAngle;
            correction = angleTwo - angleOne;
            correction *= 0.1;
            telemetry.addData("Correction", correction);

            leftFrontMotor.setPower(motorPower + correction);
            rightFrontMotor.setPower(-motorPower);
            leftBackMotor.setPower(-motorPower + correction);
            rightBackMotor.setPower(motorPower);

            telemetry.addData("Angle", angleTwo);
            telemetry.update();
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    //----------------------------------------------------------------------------------------------
    //
    /////////////////////////// diagonal drive only mecanum wheels  ////////////////////////////////
    //
    //----------------------------------------------------------------------------------------------
    void strafeDiagonal(double strafe, long time) {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(-strafe);
        leftBackMotor.setPower(-strafe);
        rightBackMotor.setPower(0);

        sleep(time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void encoderStrafeDiagnal(double strafe, int time) {

    }


    //----------------------------------------------------------------------------------------------
    //
    /////// Miscellaneous functions - should work for both normal and mecanum wheels  //////////////
    //
    //----------------------------------------------------------------------------------------------
    void turn(double motorPower, long time) {
        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(-motorPower);
        leftBackMotor.setPower(motorPower);
        rightBackMotor.setPower(-motorPower);
        sleep(time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    float performCorrection(double motorPower,float angleOne) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angleTwo = angles.firstAngle;
        telemetry.addData("Angle", angleTwo);
        telemetry.update();

        if (angleOne > angleTwo) {
            leftFrontMotor.setPower(-motorPower);
            rightFrontMotor.setPower(motorPower);
            leftBackMotor.setPower(-motorPower);
            rightBackMotor.setPower(motorPower);
            while (angleOne > angleTwo && opModeIsActive()){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleTwo = angles.firstAngle;
                telemetry.addData("Angle", angleTwo);
                telemetry.update();
            }
        } else if (angleOne < angleTwo ) {
            leftFrontMotor.setPower(motorPower);
            rightFrontMotor.setPower(-motorPower);
            leftBackMotor.setPower(motorPower);
            rightBackMotor.setPower(-motorPower);
            while (angleOne < angleTwo && opModeIsActive()){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleTwo = angles.firstAngle;
                telemetry.addData("Angle", angleTwo);
                telemetry.update();
            }
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        return angleTwo;
    }
}
