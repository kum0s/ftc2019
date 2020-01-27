package org.firstinspires.ftc.teamcode.ftcclass;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "FTCSummerClass2019Autonomous", group = "Auto")
//@Disabled
public class FTCSummerClass2019Auto extends LinearOpMode {
    //Drive
    DcMotor driveR;
    DcMotor driveL;

    @Override
    public void runOpMode() throws InterruptedException { 
        
        //initialize motors and sensors
        driveR = hardwareMap.get(DcMotor.class, "driveR");
        driveL = hardwareMap.get(DcMotor.class, "driveL");

        driveR.setDirection(DcMotor.Direction.REVERSE);
        driveL.setDirection(DcMotor.Direction.FORWARD);
        
        
        telemetry.addData("", "Ready for Start");
        telemetry.update();
        
        waitForStart();
        initializeEncoders();

        while (opModeIsActive()) {
            //Drive the robot
            driveRobot();
            sleep(3000);

            telemetry.update();
            
            break;
        }
    }

    void driveRobot(){
        moveUsingEncoder(500, 0.5);
        turnUsingEncoder(30, 0.25);
        moveUsingEncoder(100, 0.5);
    }

    void initializeEncoders(){
        driveL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void moveUsingEncoder(int motorPos, double motorPower){
        int targetPosL = driveL.getCurrentPosition() + motorPos ;
        int targetPosR = driveR.getCurrentPosition() + motorPos ;
        driveL.setTargetPosition(targetPosL);
        driveR.setTargetPosition(targetPosR);

        driveL.setPower(motorPower);
        driveR.setPower(motorPower);
        while(opModeIsActive() && driveL.isBusy() && (driveL.getCurrentPosition() < targetPosL) && driveR.isBusy() && (driveR.getCurrentPosition() < targetPosR) ) {
            //Loop body can be empty
            idle();
        }
        driveL.setPower(0);
        driveR.setPower(0);
    }

    void turnUsingEncoder(int motorPos, double motorPower){
        int targetPosL = driveL.getCurrentPosition() + motorPos ;
        int targetPosR = driveR.getCurrentPosition() - motorPos ;
        driveL.setTargetPosition(targetPosL);
        driveR.setTargetPosition(targetPosR);

        driveL.setPower(motorPower);
        driveR.setPower(motorPower);
        while(opModeIsActive() && driveL.isBusy() && (driveL.getCurrentPosition() < targetPosL) && driveR.isBusy() && (driveR.getCurrentPosition() < targetPosR) ) {
            //Loop body can be empty
            idle();
        }
        driveL.setPower(0);
        driveR.setPower(0);
    }

    void turnUsingEncoderaa(int motorPos, double motorPower){
        driveR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int targetPos = driveL.getCurrentPosition() + motorPos ;
        driveL.setTargetPosition(targetPos);

        driveL.setPower(motorPower);
        driveR.setPower(-motorPower);
        while(driveL.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            idle();
        }
        driveL.setPower(0);
        driveR.setPower(0);
    }
}
