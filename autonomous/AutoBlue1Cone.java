package org.firstinspires.ftc.bruteforce.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;


/**
 * Autonomous program starting at Blue Storage location
 */
/*@Autonomous(name="AutoBlue1Cone", group="group1")
public class AutoBlue1Cone extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        robot.moveClaw(0.7);
        
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
        

        //move clawRot to the back position at the start of the match
        clawRotState = "moving clawRot to the back position";
        telemetry.addData("State of ClawRot:", clawRotState);
        telemetry.update();
        robot.moveClawRot(1);

        //move robot forward to third tile to push the sleeve
        robotState = "moving forward";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDriveSpeed("f", 60, 0.5, 5, opModeIsActive(), telemetry); 
        
        //move robot backward so that we can place a cone
        robotState = "moving backward";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDriveSpeed("b", 16, 0.5, 5, opModeIsActive(), telemetry);
        
        //turn robot left so that it is facing the high junction
        robotState = "turning left";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDrive("rl", 10, 5, opModeIsActive(), telemetry);  
        
        //drive forward a little to the junction
        robotState = "moving forward";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDrive("f", 3.75, 5, opModeIsActive(), telemetry);

        //move clawRot to middle position so it dosen't hit the pole
        clawRotState = "moving clawRot to middle position";
        telemetry.addData("State of ClawRot:", clawRotState);
        telemetry.update();
        robot.moveClawRot(0.8);
        
        //move slider up towards the high junction
        slideState = "moving slider up";
        telemetry.addData("State of Slider:", slideState);
        telemetry.update();
        robot.encoderSlide(start - 4000);
        lastVSpos = (start - 4000);
        robot.holdViper(lastVSpos);
        
        //move clawRot to back position for optimal cone placement ontop of the junction
        clawRotState = "moving clawRot to back position";
        telemetry.addData("State of ClawRot:", clawRotState);
        telemetry.update();
        robot.moveClawRot(1);
        sleep(500);
        
        //move slider down so that the cone is resting on top of the pole
        slideState = "moving slider down";
        telemetry.addData("State of Slide:", slideState);
        telemetry.update();
        robot.encoderSlide(start - 3300);
        lastVSpos = (start - 3300);
        robot.holdViper(lastVSpos);
        sleep(500);
        
        //open claw and drop cone on the pole
        clawState = "opening claw";
        telemetry.addData("State of Claw:", clawState);
        telemetry.update();
        robot.moveClaw(open);
        sleep(500);
        
        //move slider up after the cone has been dropped
        slideState = "moving slider up";
        telemetry.addData("State of Slide:", slideState);
        telemetry.update();
        robot.encoderSlide(start - 4000);
        lastVSpos = (start - 4000);
        robot.holdViper(lastVSpos);
        
        //move clawRot to front position after the slider is up
        clawRotState = "moving clawRot to front position";
        telemetry.addData("State of ClawRot:", clawRotState);
        telemetry.update();
        robot.moveClawRot(0.3);
        sleep(500);
        //end of similarities
        
        //move slider down to get reaady to turn towards the stack
        slideState = "moving slider down";
        telemetry.addData("State of Slide:", slideState);
        telemetry.update();
        robot.encoderSlide(start + 400);
        //lastVSpos = (start + 400);
        //robot.holdViper(lastVSpos);
        
        //move clawRot to back position after the slider is up
        clawRotState = "moving clawRot to back position";
        telemetry.addData("State of ClawRot:", clawRotState);
        telemetry.update();
        robot.moveClawRot(1);
        sleep(500);
        
        //turn right towards the cone stack
        robotState = "turning left";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDrive("rr", 10, 5, opModeIsActive(), telemetry);
        
        //go back
        robotState = "moving back";
        telemetry.addData("State of Robot:", robotState);
        telemetry.update();
        robot.encoderDrive("b", 20, 5, opModeIsActive(), telemetry);
        
        
        
        
        //strafe into the correct parking based on the sleeve
        if (sleeve==1) {
            
            //strafe left to park in spot 1
            robotState = "strafing left";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("l", 27, 5, opModeIsActive(), telemetry);
            
            //move robot forward a little for safety
            robotState = "moving forwards";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("f", 5, 5, opModeIsActive(), telemetry);
            
        } else if (sleeve==2) {
            
            //stay where you are for spot 2
            robotState = "idle";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update()
          ;
        } else if (sleeve==3) {
            
            //strafe right to park in spot 3
            robotState = "strafing right";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("r", 28, 5, opModeIsActive(), telemetry);
            
            //move forward a little for safety
            robotState = "moving forward";
            telemetry.addData("State of Robot:", robotState);
            telemetry.update();
            robot.encoderDrive("f", 5, 5, opModeIsActive(), telemetry);
            
        }
        
        
        
        
        // Stop the robot
        robotState = "idle";
        slideState = "idle";
        clawRotState = "idle";
        clawState = "idle";
        robot.robotStop();
        
        is this changed
        
        telemetry.addData("Path", "Complete");
        telemetry.addData("State of Robot: ", robotState);
        telemetry.addData("State of Slider: ", slideState);
        telemetry.addData("State of Claw: ", clawState);
        telemetry.addData("State of ClawRot: ", clawRotState);
        telemetry.update();
    }
}*/