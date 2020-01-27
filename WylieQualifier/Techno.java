package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Disabled
@TeleOp(name = "Techno", group = "Teleop")
//--------------------------------------------------------------------------------------------------
// The class can be used as generic driving of robot.
// Instead of writing fresh code each time, this class can be used for driving robot.
// This class can serve as starting point for each season's Driver Controlled code.
//--------------------------------------------------------------------------------------------------
public class Techno extends RobotDrive {
    DcMotor intakeServoR;
    DcMotor intakeServoL;
    
    CRServo clawL;
    CRServo clawR;
    Servo clamp;
    
    Servo foundationL;
    Servo foundationR;
    
    DcMotor flip;
    DcMotor lift;
    
    double flipSpeed = 0.5;
    double liftSpeed = 0.5;
        
    @Override
    public void runOpMode()throws InterruptedException {
        intakeServoL = hardwareMap.dcMotor.get("intakeServoL");
        intakeServoR = hardwareMap.dcMotor.get("intakeServoR");
        intakeServoR.setDirection(DcMotor.Direction.REVERSE);
        
        foundationL = hardwareMap.get(Servo.class, "foundationL");
        foundationR = hardwareMap.get(Servo.class, "foundationR");
        foundationR.setDirection(Servo.Direction.REVERSE);
                
        flip = hardwareMap.dcMotor.get("flip");
        lift = hardwareMap.dcMotor.get("lift");
        clawR = hardwareMap.crservo.get("clawR");
        clawL = hardwareMap.crservo.get("clawL");
        clamp = hardwareMap.get(Servo.class, "clamp");
        
        flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flip.setTargetPosition(0);
        flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set encoder target positions to zero to begin with
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        initializeRobot();
        waitForStart();
        
        while (opModeIsActive()){
            mecanumDrive();
            intake();
            grabStone();
            flip();
            lift();
            foundation();
            grabStoneCombo();
            
            telemetry.addData("Flip Position: ", flip.getCurrentPosition());
            telemetry.addData("Lift Position: ", lift.getCurrentPosition());
            telemetry.update();
        }
    }
    
    void flip() {
        if (gamepad2.a) {
            flip.setTargetPosition(-3500);
            flip.setPower(flipSpeed);
            flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //use encoders to move flip to pos. 1
        } else if (gamepad2.b) {
            flip.setTargetPosition(-3100);
            flip.setPower(flipSpeed);
            flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //use encoders to move flip to pos. 2
        } else if (gamepad2.y) {
            flip.setTargetPosition(-2700);
            flip.setPower(flipSpeed);
            flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //use encoders to move flip to pos. 3
        } else if (gamepad2.x) {
            flip.setTargetPosition(-2300);
            flip.setPower(flipSpeed);
            flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //use encoders to move flip to pos. 4
        } else if (gamepad2.right_stick_button) {
            flip.setTargetPosition(0);
            flip.setPower(-0.5);
            flip.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //move flip back to pos. 0
        } else if (gamepad2.right_bumper) {
            flip.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //reset flip encoders manually
        } else if (gamepad2.right_stick_y != 0) {
            flip.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (gamepad2.right_stick_y != 0) {
                flip.setPower(gamepad2.right_stick_y*(-0.5));
            }
            flip.setPower(0);
            flip.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flip.setTargetPosition(flip.getCurrentPosition());
            flip.setPower(0.025);
            //manual mode for moving the flip
        }
    }
    
    void lift() {
        if (gamepad2.dpad_up) {
            lift.setTargetPosition(2500);
            lift.setPower(liftSpeed);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //move lift to max pos.
        } else if (gamepad2.dpad_down || gamepad2.left_stick_button) {
            lift.setTargetPosition(0);
            lift.setPower(liftSpeed);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //move lift down to zero pos.
        } else if (gamepad2.left_bumper) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //reset encoders for lift
        } else if (gamepad2.left_stick_y > 0.5 || gamepad2.left_stick_y < -0.5) {
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (gamepad2.left_stick_y != 0 && lift.getCurrentPosition() < 2500) {
                lift.setPower(gamepad2.left_stick_y*(-1));
            }
            //creates digital stopper
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(flip.getCurrentPosition());
            lift.setPower(0.1);
            //manual mode for moving lift
        }
    }
    
    void grabStone() {
        if (gamepad2.left_stick_x == -1 && gamepad2.right_stick_x == 1) {
            //open position
            clamp.setPosition(0.55);
        } else if (gamepad2.left_stick_x == 1 && gamepad2.right_stick_x == -1) {
            //closed position
            clamp.setPosition(0.47);
        }
    }
    void grabStoneCombo() {
        if (gamepad2.dpad_down) {
             clamp.setPosition(0.46);
             clawR.setPower(0.5);
             clawL.setPower(-0.5);
        } else {
            clawR.setPower(0);
            clawL.setPower(0);
        }
    }
    
    void intake() {
        if (gamepad2.left_trigger != 0) {
            intakeServoR.setPower(gamepad2.left_trigger);
            intakeServoL.setPower(gamepad2.left_trigger);
        } else if (gamepad2.right_trigger != 0) {
            intakeServoR.setPower(gamepad2.right_trigger*(-1));
            intakeServoL.setPower(gamepad2.right_trigger*(-1));
        } else {
            intakeServoR.setPower(0);
            intakeServoL.setPower(0);
        }
    }
    
    void foundation () {
        if (gamepad1.a) {
            foundationL.setPosition(0.2);
            foundationR.setPosition(0.2);
        } else if (gamepad1.y) {
            foundationL.setPosition(1);
            foundationR.setPosition(1);   
        }
    }
}
