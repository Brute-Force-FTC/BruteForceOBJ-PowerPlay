// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.bruteforce.autonomous;

// Imports all packages and files needed to run the program

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="Correct", group="group 1")
//@Disabled
public class CorrectionCode extends LinearOpMode
{
    
    // Creates all variables needed. Our 4 DcMotors, the imu, and finds the orientation of all 3 axis (xyz) on the robot. 
    // Also defines the power as 0.3, and sets the global angle to the correction. 
    // Also creates booleans for both controller buttons a and b. 
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        
        //sets all 4 motors to brake
        
        //creates parameters for the imu. 
        //Instances of Parameters contain data indicating how a BNO055 absolute orientation sensor is to be initialized.
        
        //set the parameters mode = to the sensor mode we want to use the sensor in - this case the imu sensor
        //set the parameters units in which angles and angular rates are measured = sets this to the imu's degrees
        //set the parameters units in which acceleration is measured in = sets this to meters per second
        //set the parameters debugging aid = enable logging? - yes change to true

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        
        //initializes all parameters set before

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        
        //return the calibration status of the imu and converts it to a String
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.
        
        robot.resetAngle(globalAngle);
        
        correction = robot.checkDirectionF(globalAngle, telemetry);
        //robot.encoderDrive("f", 50, 5, opModeIsActive(), telemetry);
        robot.encoderDriveIMU("f", 50, globalAngle, 5, opModeIsActive(), telemetry);
        robot.encoderDriveIMU("b", 50, globalAngle, 5, opModeIsActive(), telemetry);
        robot.encoderDriveIMU("r", 50, globalAngle, 5, opModeIsActive(), telemetry);
        robot.encoderDriveIMU("l", 50, globalAngle, 5, opModeIsActive(), telemetry);

        /*while (opModeIsActive())
        {
            // Use gyro to drive in a straight line.
            //set correction to the output of checkDirection
            correction = robot.checkDirectionF(globalAngle, telemetry);
            
            robot.encoderDrive("f", 10, 5, opModeIsActive(), telemetry);
            //robot.encoderDriveIMU("f", 10, correction, globalAngle, 5, opModeIsActive(), telemetry);
            
            sets the imu heading of the the chronologically first rotation made in the 
            //sets the imu heading to the chronological last rotation and 
            //finds the first rotation in the sequence based on the axes order (ZYX)
            telemetry.addData("1 imu heading", robot.lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();
            
            //takes the power and subtracts the correction to correct for straight movement
            robot.frontLeft.setPower(-(power - correction));
            robot.frontRight.setPower((power + correction)); //+
            robot.backLeft.setPower(-(power - correction));
            robot.backRight.setPower((power + correction)); //+
            
            sleep(5000);
            
            frontLeft.setPower(0);
            frontRight.setPower(0); //+
            backLeft.setPower(0);
            backRight.setPower(0);
            
            resetAngle();
            
            correction = checkDirectionB();
            
            frontLeft.setPower((power - correction));
            frontRight.setPower(-(power + correction)); //+
            backLeft.setPower((power - correction));
            backRight.setPower(-(power + correction));
            
            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.
        }*/

        // turn the motors off.
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    
    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    /*private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        robot.resetAngle(globalAngle);

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.frontLeft.setPower(leftPower);
        robot.frontRight.setPower(rightPower);
        robot.backLeft.setPower(leftPower);
        robot.backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.getAngle(globalAngle) == 0) {}

            while (opModeIsActive() && robot.getAngle(globalAngle) > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && robot.getAngle(globalAngle) < degrees) {}

        // turn the motors off.
        robot.frontRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.backLeft.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        robot.resetAngle(globalAngle);
    }*/
}
