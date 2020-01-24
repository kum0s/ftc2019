package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "Techno", group = "Teleop")
//--------------------------------------------------------------------------------------------------
// The class can be used as generic driving of robot.
// Instead of writing fresh code each time, this class can be used for driving robot.
// This class can serve as starting point for each season's Driver Controlled code.
//--------------------------------------------------------------------------------------------------
public class Techno extends RobotDrive {
    @Override
    public void runOpMode()throws InterruptedException {

        initializeRobot();
        waitForStart();
        
        while (opModeIsActive()){
            mecanumDrive();
        }
    }

}
