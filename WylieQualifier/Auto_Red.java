package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@Autonomous(name = "Auto_Red", group = "Autonomous")
//--------------------------------------------------------------------------------------------------
// The class can be used as generic driving of robot.
// Instead of writing fresh code each time, this class can be used for driving robot.
// This class can serve as starting point for each season's Driver Controlled code.
//--------------------------------------------------------------------------------------------------
public class Auto_Red extends RobotDrive {
        
    @Override
    public void runOpMode()throws InterruptedException {
        
        ColorSensor colorL;
        ColorSensor colorR;
        
        colorL = hardwareMap.colorSensor.get("colorL");
        colorR = hardwareMap.colorSensor.get("colorR");
        
        Servo foundationL;
        Servo foundationR;
        
        foundationL = hardwareMap.get(Servo.class, "foundationL");
        foundationR = hardwareMap.get(Servo.class, "foundationR");
        foundationR.setDirection(Servo.Direction.REVERSE);
        
        DcMotor intakeServoR;
        DcMotor intakeServoL;
        
        intakeServoL = hardwareMap.dcMotor.get("intakeServoL");
        intakeServoR = hardwareMap.dcMotor.get("intakeServoR");
        intakeServoR.setDirection(DcMotor.Direction.REVERSE);
    
        int rgbR = colorR.red()*colorR.green()*colorR.blue();
        int rgbL = colorL.red()*colorL.green()*colorL.blue();
        
        int position = 0;
        
        initializeRobot();
        initializeEncoders();
        initializeIMU();
    
        waitForStart();
        
        while (opModeIsActive()){
            // telemetry.addData("RedR: ", colorR.red());
            // telemetry.addData("GreenR: ", colorR.green());
            // telemetry.addData("BlueR", colorR.blue());
            
            // telemetry.addData("RedL: ", colorL.red());
            // telemetry.addData("GreenL: ", colorL.green());
            // telemetry.addData("BlueL: ", colorL.blue());
            
            // telemetry.addData("LuminosityR: ", colorR.alpha());
            // telemetry.addData("LuminosityL: ", colorL.alpha());
            
            // telemetry.addData("rgbR: ", (colorR.red()*colorR.green()*colorR.blue()));
            // telemetry.addData("rgbL: ", (colorL.red()*colorL.green()*colorL.blue()));
            
            // telemetry.addData("argbR: ", colorR.argb());
            // telemetry.addData("argbL: ", colorL.argb());
            
            // telemetry.update();
            
            //Unlock Foundation
            foundationR.setPosition(1);
            foundationL.setPosition(1);
            sleep(250);
            
            //Move Forward
            moveUsingEncoder(-925, 0.4, 0);
            sleep(100);
            
            //Perform Correction
            performCorrection(0.1, 0);
            sleep(250);
            
            sleep(100);
            
            //Robot Adjustment based on Distance
            while (colorL.alpha() < 68 && colorR.alpha() < 68 && opModeIsActive()) {
                moveUsingEncoder(-25, 0.5, 0);
            }
            
            while (colorL.alpha() > 70 && colorR.alpha() > 70 && opModeIsActive()) {
                moveUsingEncoder(25, 0.5, 0);
            }
            
            //Perform Correction
            performCorrection(0.18, 0);
            
            sleep(100);
            
            telemetry.addData("", colorL.red());
            telemetry.addData("", colorR.red());
            
            int redL = colorL.red();
            int redR = colorR.red();
            
            
            if (redL > 25 && redR > 25) {
                //Middle Block
                position = 2;
                telemetry.addData("Middle", 1);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
                
                //Move Forward
                moveUsingEncoder(-200, 0.1, 0);
               
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                
                sleep(100);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
               
                //Move Forward
                moveUsingEncoder(-200, 0.1, 0);
                
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                
                sleep(100);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
                
                //Move Foward
                moveUsingEncoder(-400, 0.1, 0);
                
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                sleep(100);
                
                //Move Backwards
                moveUsingEncoder(950, 0.5);
                sleep(100);
                
                //Perform Correction
                performCorrection(0.18, 0);
                
                //Strafe Right
                encoderStrafe(-400, 0.5);
                
            } else if (redL > 25 && redR < 25) {
                //Right Block
                position = 1;
                telemetry.addData("Right", 1);
                
                //Strafe Right
                encoderStrafe(-400, 0.2);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
                
                //Move Forward
                moveUsingEncoder(-200, 0.1, 0);
               
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                
                sleep(100);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
               
                //Move Forward
                moveUsingEncoder(-200, 0.1, 0);
                
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                
                sleep(100);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
                
                //Move Foward
                moveUsingEncoder(-400, 0.1, 0);
                
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                sleep(100);
                
                //Move Backwards
                moveUsingEncoder(950, 0.5);
                sleep(100);
                
                //Perform Correction
                performCorrection(0.18, 0);
                
            } else {
                //Left Block
                position = 3;
                telemetry.addData ("Left", 1);
               
                //Strafe Left
                encoderStrafe(400, 0.2);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
                
                //Move Forward
                moveUsingEncoder(-200, 0.1, 0);
               
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                
                sleep(100);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
               
                //Move Forward
                moveUsingEncoder(-200, 0.1, 0);
                
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                
                sleep(100);
                
                //start intake
                intakeServoR.setPower(-0.5);
                intakeServoL.setPower(-0.5);
                
                //Move Foward
                moveUsingEncoder(-400, 0.1, 0);
                
                //stop intake
                intakeServoR.setPower(0);
                intakeServoL.setPower(0);
                sleep(100);
                
                //Move Backwards
                moveUsingEncoder(950, 0.5);
                sleep(100);
                
                //Perform Correction
                performCorrection(0.18, 0);
                
                //Strafe Right
                encoderStrafe(-800, 0.5);
            }
            telemetry.update();

            sleep(100);
            
            //Strafe Right
            encoderStrafe(-2550, 0.5, 0);
            
            //Perform Correction
            performCorrection(0.2, 0);
            
            //Move Forward
            moveUsingEncoder(-450, 0.5, 0);
            
            //Spin the Block Out
            intakeServoR.setPower(1);
            intakeServoL.setPower(1);
            
            //Move Backwards
            moveUsingEncoder(300, 0.5, 0);
            
            sleep(500);
            
            //Stop Intake Spin
            intakeServoR.setPower(0);
            intakeServoL.setPower(0);
            
            performCorrection(0.4, 90);
            sleep(250);
            moveUsingEncoder(-2400, 0.75, 90);
            performCorrection(0.15, 90);
            sleep(100);
            
            if (position == 1) {
                moveUsingEncoder(-150, 0.75, 90);
                encoderStrafe(-1300, 0.5, 90);
                sleep(100);
                intakeServoR.setPower(-1);
                intakeServoL.setPower(-1);
                moveUsingEncoder(-400, 0.5, 90);
                sleep(100);
                encoderStrafe(1500, 0.5, 90);
                performCorrection(0.3, 90);
                // intakeServoR.setPower(0);
                // intakeServoL.setPower(0);
                
            } else if (position == 2) {
                moveUsingEncoder(-300, 0.5, 90);
                encoderStrafe(-1500, 0.5, 90);
                sleep(100);
                intakeServoR.setPower(-1);
                intakeServoL.setPower(-1);
                moveUsingEncoder(-400, 0.5, 90);
                sleep(100);
                encoderStrafe(1500, 0.5, 90);
                performCorrection(0.3, 90);
                // intakeServoR.setPower(0);
                // intakeServoL.setPower(0);
                moveUsingEncoder(550, 0.5, 90);
            } else {
                moveUsingEncoder(-700, 0.5, 90);
                encoderStrafe(-1500, 0.5, 90);
                sleep(100);
                intakeServoR.setPower(-1);
                intakeServoL.setPower(-1);
                moveUsingEncoder(-400, 0.5, 90);
                sleep(100);
                encoderStrafe(1300, 0.5, 90);
                performCorrection(0.3, 90);
                // intakeServoR.setPower(0);
                // intakeServoL.setPower(0);
                moveUsingEncoder(700, 0.5, 90);
            }
            
            intakeServoR.setPower(0);
            intakeServoL.setPower(0);
            moveUsingEncoder(2400, 0.75, 90);
            performCorrection(0.75, -45);
            intakeServoR.setPower(1);
            intakeServoL.setPower(1);
            moveUsingEncoder(750, 0.75, -90);
            
            // //Strafe Left
            // encoderStrafe(1800, 0.6, 0);
            // sleep(100);
            
            // //Perform Correction
            // performCorrection(0.18, 0);
            
            // //Move Forward for the Foundation
            // moveUsingEncoder(-600, 0.3, 0);
            // moveUsingEncoder(-150, 0.2, 0);
            // sleep(500);
            
            // //Lock Foundation
            // foundationR.setPosition(0.2);
            // foundationL.setPosition(0.2);
            // sleep(500);
            
            // //Move Backwards
            // moveUsingEncoder(1375, 0.4);
            // sleep(100);
            
            // //Unlock Foundation
            // foundationR.setPosition(1);
            // foundationL.setPosition(1);
            // sleep(250);
            
            // //Strafe Right
            // encoderStrafe(-2000, 0.6, 0);
            // sleep(100);
            
            // //Move Forward
            // moveUsingEncoder(-1000, 0.6, 0);
            
            // //Turn Using Perform Correction 
            // performCorrection(0.3, -90);
            
            // //Move Backwards to Push Foundation
            // moveUsingEncoder(600, 1, -90);
            
            // //Perform Correction
            // performCorrection(0.3, -90);
            
            // //Move Forward Under the Bridge
            // moveUsingEncoder(-700, 1, -90);
            
            // //Stop
            break;
        }
        
        
    }
    
   
}
