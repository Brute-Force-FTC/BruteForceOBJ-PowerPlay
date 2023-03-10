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
public class clawRotTest extends LinearOpMode {

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
        
        //robot.encoderSlide(-100);
        //robot.encoderSlide2(100);
        double lastVSpos = 0;
        double lastVSpos2 = 0;
        
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
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        // the variable for Right Stick X, Right Stick Y, Left Stick X, Left Stick Y are defined
        
        robot.moveClaw(0.9);
        boolean flipped = false;
        double holdingRot = -0.001;
        boolean holdingSlide = false;
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
                robot.moveSlider(lsy2*0.1);
                } else if (lsy2 < 0) {
                robot.moveSlider(lsy2);
                } else {
                    if (holdingSlide == false) {
                        robot.moveSlider(-0.005);
                        holdingSlide = true;
                    }
                }
            
            
            
            if (rsy2 > 0) {
                robot.moveClawRot(-rsy2);
            } else if (rsy2 < 0) {
                robot.moveClawRot(-rsy2);
            } else {
                robot.moveClawRot(holdingRot);
            }
            
            if(robot.clawRot.getCurrentPosition() > 2000){
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
                
                
        //program to toggle the claw and clawRot using the right and left bumper on controller 2
            if (robot.getClaw().getPower() < 0.91 && robot.getClaw().getPower() > 0.89 && gamepad2.right_bumper) {
                robot.moveClaw(0.4);
                sleep(200);
            } else if (robot.getClaw().getPower() > 0.39 && robot.getClaw().getPower() < 0.41 && gamepad2.right_bumper) {
                robot.moveClaw(0.9);
                sleep(200);
            }
            
            
            
            // telemetry is added for the 4 stick values
            
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
            telemetry.addData("Status", "Running"); 
            telemetry.update();
            
            
        }
    }
    
}


