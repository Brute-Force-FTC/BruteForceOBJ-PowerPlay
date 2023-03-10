/*
Copyright 2021 FIRST Tech Challenge Team FTC
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
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

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp
public class PID extends LinearOpMode {

//Runs Tele-Op Mode
//Main program that has the logic to run the missions.

//create all the vars for the controller sticks
double rsy1 = 0;
double rsx1 = 0;
double lsx1 = 0;
double lsy1 = 0;
double rsy2 = 0;
double rsx2 = 0;
double lsx2 = 0;
double lsy2 = 0;

//get the battery voltage
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

    @Override
    public void runOpMode() {
       
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
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d", robot.getFrontLeft().getCurrentPosition(), robot.getFrontRight().getCurrentPosition(), robot.getBackLeft().getCurrentPosition(), robot.getBackRight().getCurrentPosition());
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        // the variable for Right Stick X, Right Stick Y, Left Stick X, Left Stick Y are defined
        int begin = robot.viperSlide.getCurrentPosition();
        
        String robotState = "idle";
        String slideState = "idle";
        String clawRotState = "idle";
        String clawState = "idle";
        
        
        int start = begin;
        
        double lastVSpos = (start - 50);
        
        
        double close = 0.7;
        double open = -0.5;
        boolean clawPos = false;
        robot.moveClawRot(0.8);
        robot.moveClaw(0.7);
        
        
        while (opModeIsActive()) {
            // the controller movements are assigned to the variables
            rsy1 = this.gamepad1.right_stick_y;
            rsx1 = this.gamepad1.right_stick_x;
            lsx1 = this.gamepad1.left_stick_x;
            lsy1 = this.gamepad1.left_stick_y;
            rsy2 = this.gamepad2.right_stick_y;
            rsx2 = this.gamepad2.right_stick_x;
            lsx2 = this.gamepad2.left_stick_x;
            lsy2 = this.gamepad2.left_stick_y;
            // The variables are assigned to the robot movement functions
            
            
            if (lsy2 > 0) {
                robot.moveSlider(0.7*lsy2);
                } else if (lsy2 < 0) {
                robot.moveSlider(lsy2);
                } else {
                robot.stopSlider();
                }


            double Kp = 0.002;
            double Ki = 0; //0.0004 or 0.0003
            double Kd = 0; //0.000025 or 0.0002

            //double derivative = 0;
            double integralSum = 0;

            double lastError = 0; 
            //double error = 0;
            //double out = 0;

            ElapsedTime timer = new ElapsedTime();

            //keeps the slider at the position and resistant to gravity
            if (lsy2 == 0) {
                    if (robot.viperSlide.getCurrentPosition() != lastVSpos) {

                        double error = lastVSpos - robot.viperSlide.getCurrentPosition();

                        double derivative = (error - lastError) / timer.seconds();

                        integralSum = integralSum + (error * timer.seconds());

                        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                        robot.viperSlide.setPower(out);

                        lastError = error;

                        timer.reset();
                    }
                } else {
                    lastVSpos = robot.viperSlide.getCurrentPosition();
                }



            if (rsy2 < 0) {
                sleep(100);
                clawRotState = "moving up";
                //telemetry.addData("State of clawRot:", clawRotState);
                //telemetry.update();
                robot.moveClawRot(robot.clawRot.getPower() + 0.05);
                if (robot.clawRot.getPower() <= 0.1) {
                    robot.moveClawRot(0.15);
                } else if (robot.clawRot.getPower() >= 1) {
                    robot.moveClawRot(0.9);
                }
            } else {
                clawRotState = "idle";
                //telemetry.addData("State of clawRot:", clawRotState);
                //telemetry.update();
                ;
            }
            
            
            //program to toggle the claw and clawRot using the right and left bumper on controller 2
            if (robot.getClaw().getPower() < 0.71 && robot.getClaw().getPower() > 0.69 && gamepad2.right_bumper) {
                clawState = "closing";
                //telemetry.addData("State of claw:", clawState);
                //telemetry.update();
                robot.moveClaw(-0.5);
                sleep(200);
            } else if (robot.getClaw().getPower() < -0.49 && robot.getClaw().getPower() > -0.51 && gamepad2.right_bumper) {
                clawState = "opening";
                //telemetry.addData("State of claw:", clawState);
                //telemetry.update();
                robot.moveClaw(0.7);
                sleep(200);
            }
            
            
            if (robot.getClawRot().getPower() < 0.16 && robot.getClawRot().getPower() > 0.14 && gamepad2.left_bumper) {
                clawRotState = "moving to top position";
                //telemetry.addData("State of clawRot:", clawRotState);
                //telemetry.update();
                robot.moveClawRot(1);
                sleep(200);
            } else if (robot.getClawRot().getPower() == 1 && gamepad2.left_bumper) {
                clawRotState = "moving to bottom position";
                //telemetry.addData("State of clawRot:", clawRotState);
                //telemetry.update();
                robot.moveClawRot(0.15);
                sleep(200);
            } else if (robot.getClawRot().getPower() < 0.81 && robot.getClawRot().getPower() > 0.79 && gamepad2.left_bumper) {
                clawRotState = "moving to top position";
                //telemetry.addData("State of clawRot:", clawRotState);
                //telemetry.update();
                robot.moveClawRot(1);
                sleep(200);
            } else if (gamepad2.b) {
                clawRotState = "moving to middle position";
                //telemetry.addData("State of clawRot:", clawRotState);
                //telemetry.update();
                robot.moveClawRot(0.8);
            }
            
            
            
            
          //Shows Values on Driver hub  
            telemetry.addData("Right Stick Y 1", rsy1);
            telemetry.addData("Right Stick X 1", rsx1);                         
            telemetry.addData("Left Stick X 1", lsx1);
            telemetry.addData("Left Stick Y 1", lsy1);
            telemetry.addData("Right Stick Y 2", rsy2);
            telemetry.addData("Right Stick X 2", rsx2);
            telemetry.addData("Left Stick X 2", lsx2);
            telemetry.addData("Left Stick Y 2", lsy2);
            telemetry.addData("Servo Speed", robot.getClaw().getPower());
            telemetry.addData("ServoRot Speed", robot.getClawRot().getPower());
            telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d :%7d", robot.frontLeft.getCurrentPosition(), robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition(), robot.viperSlide.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            
        }
    }
    
}