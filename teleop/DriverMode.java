// simple teleop program that drives bot using controller joysticks in tank mode.
// this code monitors the period and stops when the period is ended.

package org.firstinspires.ftc.bruteforce.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.Func;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.PrintStream;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver Mode", group="Exercises")
//@Disabled
public class DriverMode extends LinearOpMode
{
        double rsy1 = 0;
        double rsx1 = 0;
        double lsx1 = 0;
        double lsy1 = 0;
        double rsy2 = 0;
        double rsx2 = 0;
        double lsx2 = 0;
        double lsy2 = 0;

double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public DriverMode() throws Exception
    {
        RobotLog.d("Starting DriveTankMT");
    }

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        
        
        //new
        //robot.encoderSlide(0);
        
        telemetry.addData("voltage", "%.1f volts", new Func<Double>() {
            @Override public Double value() {
                return getBatteryVoltage();
            }
            });
            
        telemetry.addData("Status", "Initialized");
        RobotLog.d("SomeUsefulPrefixHere:Some Useful Message Here");
        telemetry.update();
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        
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
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", robot.getFrontLeft().getCurrentPosition(), robot.getFrontRight().getCurrentPosition(), robot.getBackLeft().getCurrentPosition(), robot.getBackRight().getCurrentPosition());
        int startSlide1 = robot.viperSlide.getCurrentPosition();
        int startSlide2 = robot.viperSlide2.getCurrentPosition();
        int startRot = robot.clawRot.getCurrentPosition();
        telemetry.addData("Start of Slide 1&2 and ClawRot:", "Starting at %7d :%7d :%7d", startSlide1, startSlide2, startRot);
        telemetry.update();

        // create an instance of the DriveThread.

        Thread  driveThread = new DriveThread();
        Thread slideThread = new SlideThread();

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        RobotLog.d("wait for start");

        waitForStart();
        
        String robotState = "idle";
        String slideState = "idle";
        String clawRotState = "idle";
        String clawState = "idle";
        //new
        
        double close = -0.7;
        double open = 0.2;
        boolean clawPos = false;
        RobotLog.d("ClawRot Malfunction; ClawRot Curr Pos = %f", robot.getClawRot().getPower());
        
        robot.moveClaw(close);
        boolean flipped = false;
        double holdingRot = 0.0005;
        
        RobotLog.d("started");

        // start the driving thread.

        driveThread.start();
        slideThread.start();

        // continue with main thread.

        try
        {
            while (opModeIsActive())
            {
                
            /*while (robot.clawRot.getCurrentPosition() < (startRot)) {
                robot.encoderRot(startRot+50);
            }*/
            
            if (gamepad2.left_bumper) {
                if (rsy2 > 0) {
                clawRotState = "moving down";
                //telemetry.addData("State of Slide:", slideState);
                //telemetry.update();
                robot.moveClawRot((-rsy2)*0.25);
                } else if (rsy2 < 0) {
                clawRotState = "moving up";
                //telemetry.addData("State of Slide:", slideState);
                //telemetry.update();
                robot.moveClawRot((-rsy2)*0.25);
                } else {
                clawRotState = "holding";
                //telemetry.addData("State of Slide:", slideState);
                //telemetry.update();
                robot.moveClawRot(holdingRot);
                }
            } else {
                if (rsy2 > 0) {
                clawRotState = "moving down";
                //telemetry.addData("State of Slide:", slideState);
                //telemetry.update();
                robot.moveClawRot(-rsy2);
                } else if (rsy2 < 0) {
                clawRotState = "moving up";
                //telemetry.addData("State of Slide:", slideState);
                //telemetry.update();
                robot.moveClawRot(-rsy2);
                } else {
                clawRotState = "holding";
                //telemetry.addData("State of Slide:", slideState);
                //telemetry.update();
                robot.moveClawRot(holdingRot);
                }
            }
            
            
            
                
                if (robot.clawRot.getCurrentPosition() < startRot) {
                    robot.encoderRot(startRot + 50);
            }
            
            //program to toggle the claw and clawRot using the right and left bumper on controller 2
            if (robot.getClaw().getPower() < (close+0.01) && robot.getClaw().getPower() > (close-0.01) && gamepad2.right_bumper) {
                clawState = "closing";
                //telemetry.addData("State of claw:", clawState);
                //telemetry.update();
                robot.moveClaw(open);
                sleep(200);
            } else if (robot.getClaw().getPower() > (open-0.01) && robot.getClaw().getPower() < (open+0.01) && gamepad2.right_bumper) {
                clawState = "opening";
                //telemetry.addData("State of claw:", clawState);
                //telemetry.update();
                robot.moveClaw(close);
                sleep(200);
            }
            
            if(robot.clawRot.getCurrentPosition() < 2000){
                if(flipped == true){
                    holdingRot = -holdingRot;
                    flipped = false;
                }
            }else{
                if(flipped == false){
                    holdingRot = -holdingRot;
                    flipped = true;
                }
            }
                
            telemetry.addData("Mode", "running");
            telemetry.addData("Run Time", this.getRuntime());
            telemetry.addData("State of Robot: ", robotState);
            telemetry.addData("State of Slider: ", slideState);
            telemetry.addData("State of Claw: ", clawState);
            telemetry.addData("State of ClawRot: ", clawRotState);
            telemetry.addData("Right Stick Y 1", rsy1);
            telemetry.addData("Right Stick X 1", rsx1);                         
            telemetry.addData("Left Stick X 1", lsx1);
            telemetry.addData("Left Stick Y 1", lsy1);
            telemetry.addData("Right Stick Y 2", rsy2);
            telemetry.addData("Right Stick X 2", rsx2);
            telemetry.addData("Left Stick X 2", lsx2);
            telemetry.addData("Left Stick Y 2", lsy2);
            telemetry.addData("Claw Speed", robot.getClaw().getPower());
            telemetry.addData("ClawRot Speed", robot.getClawRot().getPower());
            telemetry.addData("clawPos:", clawPos);
            telemetry.addData("Motors Encoder", "Running at %7d :%7d :%7d :%7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
            telemetry.addData("Slide Encoders 1&2","Running at %7d :%7d", robot.viperSlide.getCurrentPosition(), robot.viperSlide2.getCurrentPosition());
            telemetry.addData("ClawRot Encoder", "Running at %7d", robot.clawRot.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
                
                idle();
            }
        }
        catch(Exception e) {RobotLog.d(e.getMessage());}

        RobotLog.d("out of while loop");

        // stop the driving thread.

        driveThread.interrupt();
        slideThread.interrupt();

        RobotLog.d("end");
    }
    
    

    private class DriveThread extends Thread
    {
        
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        
        
        
        public DriveThread()
        {
            this.setName("DriveThread");
            RobotLog.d("%s", this.getName());
            
        }
        
        
        

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            
            
            RobotLog.d("Starting thread %s",this.getName());
            

            try
            {
                
                while (!isInterrupted())
                {
                    
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsx1 = gamepad1.left_stick_x;
                    lsy1 = gamepad1.left_stick_y;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;
                    lsx2 = gamepad2.left_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    
                    if (gamepad1.right_bumper) {
                if (rsy1 > 0) {
                robot.moveVertical(0.333*rsy1);
                } else if (rsy1 < 0) {
                robot.moveVertical(0.333*rsy1);
                } else if (lsx1 > 0) {
                robot.rotate(0.333*lsx1);
                } else if (lsx1 < 0) {
                robot.rotate(0.333*lsx1);
                } else {
                robot.stopMovement();
                }
            } else {
                if (rsy1 > 0) {
                robot.moveVertical(rsy1);
                } else if (rsy1 < 0) {
                robot.moveVertical(rsy1);
                } else if (lsx1 > 0) {
                robot.rotate(lsx1);
                } else if (lsx1 < 0) {
                robot.rotate(lsx1);
                } else {
                robot.stopMovement();
                }
            }
            
            while (gamepad1.dpad_right) {
                robot.moveHorizontal(1);
            } 
            while (gamepad1.dpad_left) {
                robot.moveHorizontal(-1);
            }
            
            
            
                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {e.printStackTrace();}

            RobotLog.d("end of thread %s", this.getName());
        }
    }
    
    private class SlideThread extends Thread
    {
        
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        int startSlide1 = robot.viperSlide.getCurrentPosition();
        int startSlide2 = robot.viperSlide2.getCurrentPosition();
        int startRot = robot.clawRot.getCurrentPosition();
        
        
        
        public SlideThread()
        {
            this.setName("SlideThread");
            RobotLog.d("%s", this.getName());
            
        }
        
        
        

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            
            
            RobotLog.d("Starting thread %s",this.getName());
            double lastVSpos = 0;
            double lastVSpos2 = 0;

            try
            {
                
                while (!isInterrupted())
                {
                    
                    
                    rsy1 = gamepad1.right_stick_y;
                    rsx1 = gamepad1.right_stick_x;
                    lsx1 = gamepad1.left_stick_x;
                    lsy1 = gamepad1.left_stick_y;
                    rsy2 = gamepad2.right_stick_y;
                    rsx2 = gamepad2.right_stick_x;
                    lsx2 = gamepad2.left_stick_x;
                    lsy2 = gamepad2.left_stick_y;
                    
                    String robotState = "idle";
                    String slideState = "idle";
                    String clawRotState = "idle";
                    String clawState = "idle";
                    
                    // we record the Y values in the main class to make showing them in telemetry
                    // easier.
                    
                    if (lsy2 > 0) {
                
                robot.moveSlider(lsy2*0.6);
                
            } else if (lsy2 < 0) {
                
                robot.moveSlider(lsy2);
                
            } else {
                //robot.stopSlider();
                
                robot.moveSlider(-0.0005);
                
                /*if (onceSlide == false) {
                    robot.moveSlider(holdingSlide);
                    onceSlide = true;
                }*/
                
            }
            
            while (robot.viperSlide.getCurrentPosition() > (startSlide1) || robot.viperSlide2.getCurrentPosition() < (startSlide2)) {
                robot.encoderSlideBothSpeed((-(startSlide1-25)), startSlide2+25, 0.25);
            }
            
            
            
            
                
                
            
            double Kp = 0.0018; //0.002
            double Ki = 0.00218; //0.0004 or 0.0003
            double Kd = 0.000025; //0.000025 or 0.0002
            
            /*if (lsy2 != 0) {
                    lastVSpos = robot.viperSlide.getCurrentPosition();
                    lastVSpos2 = robot.viperSlide2.getCurrentPosition();
                }
                
            if (lsy2 == 0) {
                robot.holdViper(Kp, Ki, Kd, lastVSpos, lastVSpos2);
            }*/
            
            
            
                    idle();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            // an error occurred in the run loop.
            catch (Exception e) {e.printStackTrace();}

            RobotLog.d("end of thread %s", this.getName());
        }
    }
}
