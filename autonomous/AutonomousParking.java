package org.firstinspires.ftc.bruteforce.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;


/**
 * Autonomous program starting at Blue Storage location
 */
@Autonomous(name="AutoPark", group="group1")
public class AutonomousParking extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        
        TensorFlowUtils utils = new TensorFlowUtils(hardwareMap);
        
        // Send telemetry message to signify robot waiting;
        String robotState = "idle";
        String slideState = "idle";
        String clawState = "idle";
        String clawRotState = "idle";
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.addData("State of Robot: ", robotState);
        telemetry.addData("State of Slider: ", slideState);
        telemetry.addData("State of Claw: ", clawState);
        telemetry.addData("State of ClawRot: ", clawRotState);
        telemetry.update();
    
        // make a method for resetting and running encoders
        robot.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d :%7d", robot.getFrontLeft().getCurrentPosition(), robot.getFrontRight().getCurrentPosition(), robot.getBackLeft().getCurrentPosition(), robot.getBackRight().getCurrentPosition(), robot.getSlide().getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        // Wait for TensorFlow to initialize
        sleep(5000);
        telemetry.addData(">****************", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        int sleeve = utils.returnTSEPosition(opModeIsActive(), telemetry);
        telemetry.addData("Sleeve = ", sleeve);
        telemetry.update();
        
        double close = 0.7;
        double open = -0.5;
        
        
        int start = 0;
        int finish = -4400;
        double lastVSpos = (start - 50);

        boolean clawPos = false;
        

        //close claw
        robot.moveClaw(0.4);

        //move robot forward to third tile to push the sleeve
        robotState = "moving forward";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDriveSpeed("b", 23, 0.5, 5, opModeIsActive(), telemetry); 
        
        
        //strafe into the correct parking based on the sleeve
        if (sleeve==1) {
            
            //strafe left to park in spot 1
            robotState = "strafing left";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("r", 27, 5, opModeIsActive(), telemetry);
            
            //move robot forward a little for safety
            robotState = "moving forwards";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("b", 5, 5, opModeIsActive(), telemetry);
            
        } else if (sleeve==2) {
            
            //stay where you are for spot 2
            robotState = "idle";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            
            //move robot forward a little for safety
            robotState = "moving forwards";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("b", 5, 5, opModeIsActive(), telemetry);
          
        } else if (sleeve==3) {
            
            //strafe right to park in spot 3
            robotState = "strafing right";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("l", 28, 5, opModeIsActive(), telemetry);
            
            //move forward a little for safety
            robotState = "moving forward";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("b", 5, 5, opModeIsActive(), telemetry);
            
        }
        
        
        
        
        // Stop the robot
        robotState = "idle";
        slideState = "idle";
        clawRotState = "idle";
        clawState = "idle";
        robot.robotStop();
        
        telemetry.addData("Path", "Complete");
        telemetry.addData("State of Robot: ", robotState);
        telemetry.addData("State of Slider: ", slideState);
        telemetry.addData("State of Claw: ", clawState);
        telemetry.addData("State of ClawRot: ", clawRotState);
        telemetry.update();
    }
}