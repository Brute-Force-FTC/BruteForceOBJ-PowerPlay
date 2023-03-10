package org.firstinspires.ftc.bruteforce.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;


/**
 * Autonomous program starting at Blue Storage location
 */
@Autonomous(name="TestCode", group="group1")
public class TestCode extends LinearOpMode {
    
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
        robot.getSlide2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getClawRot().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getSlide2().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getClawRot().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        

        robot.encoderRot(1000);
        //robot.encoderRot2(500);
        
        //robot.encoderRot(-500);  holds bottom side
        //robot.encoderRot2(-500);
        
        //robot.encoderRot(500);        holds top side
        //robot.encoderRot2(500);
        
        //robot.encoderRot(500);           weak hold top side
        //robot.encoderRot2(-500);
        
        //robot.encoderRot(-500);
        
        telemetry.addData("Path", "Complete");
        telemetry.addData("State of Robot: ", robotState);
        telemetry.addData("State of Slider: ", slideState);
        telemetry.addData("State of Claw: ", clawState);
        telemetry.addData("State of ClawRot: ", clawRotState);
        telemetry.update();
    }
}
