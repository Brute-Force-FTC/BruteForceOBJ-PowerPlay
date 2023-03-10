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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="NoDrift", group="group 1")
//@Disabled
public class NoDrift extends LinearOpMode
{
    // Creates all variables needed. Our 4 DcMotors, the imu, and finds the orientation of all 3 axis (xyz) on the robot. 
    // Also defines the power as 0.3, and sets the global angle to the correction. 
    // Also creates booleans for both controller buttons a and b. 
    DcMotor                 frontLeft, frontRight, backLeft, backRight;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Gets the hardware map for all 4 motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        
        //sets all 4 motors to brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //creates parameters for the imu. 
        //Instances of Parameters contain data indicating how a BNO055 absolute orientation sensor is to be initialized.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        //set the parameters mode = to the sensor mode we want to use the sensor in - this case the imu sensor
        //set the parameters units in which angles and angular rates are measured = sets this to the imu's degrees
        //set the parameters units in which acceleration is measured in = sets this to meters per second
        //set the parameters debugging aid = enable logging? - yes change to true
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        //initializes all parameters set before
        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        
        //return the calibration status of the imu and converts it to a String
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.
        
        

        while (opModeIsActive())
        {
            //possibly STRAFEFEEEEEEEEEE
            // Use gyro to drive in a straight line.
            //set correction to the output of checkDirection
            correction = checkDirectionB();
            
            //sets the imu heading of the the chronologically first rotation made in the 
            //sets the imu heading to the chronological last rotation and 
            //finds the first rotation in the sequence based on the axes order (ZYX)
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.update();
            
            //takes the power and subtracts the correction to correct for straight movement
            frontLeft.setPower((power - correction));
            frontRight.setPower((power - correction)); //+
            backLeft.setPower(-(power - correction));
            backRight.setPower(-(power + correction)); //+
            
            /*sleep(5000);
            
            frontLeft.setPower(0);
            frontRight.setPower(0); //+
            backLeft.setPower(0);
            backRight.setPower(0);
            
            resetAngle();
            
            correction = checkDirectionB();
            
            frontLeft.setPower((power - correction));
            frontRight.setPower(-(power + correction)); //+
            backLeft.setPower((power - correction));
            backRight.setPower(-(power + correction));*/
            

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            aButton = gamepad1.a;
            bButton = gamepad1.b;
            
            if (gamepad1.x) {
                correction = checkDirectionB();
                frontLeft.setPower((power - correction));
                frontRight.setPower(-(power + correction)); //+
                backLeft.setPower((power - correction));
                backRight.setPower(-(power + correction));
            }

            if (aButton || bButton)
            {
                // backup.
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);

                sleep(500);

                // stop.
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);

                // turn 90 degrees right.
                if (aButton) rotate(-90, power);

                // turn 90 degrees left.
                if (bButton) rotate(90, power);
            }
        }

        // turn the motors off.
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        //Returns the absolute orientation of the sensor as a set three angles with indicated parameters
        //Sets the Axes to INTRINSIC, meaning the axes move with the object that is rotating
        //EXTRINSIC means the axes remain fixed in the world around the object
        //Sets the order of Axes to Z, Y, and X
        //Sets the unit of measurment of the angles to degrees instead of radians
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        //sets the global angle to 0
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        
        //returns absolute orientation of sensor as var, angles
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        //creates the var deltaAngle which takes the 
        //var angles, first movement - the last recorded angle before reset
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        telemetry.addData("delta", deltaAngle);
        
        //if the delta angles diff. is above 180 degrees, then it adds 360
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        
        //ads the deltaAngle to the globalAngle
        globalAngle += deltaAngle;
        
        //sets the lastAngle to the last orientation it got previously
        lastAngles = angles;
        
        //returns the finished globalAngle
        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirectionF()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;
        
        //sets the current angle to the new globalAngle after deltaAngle has been added
        angle = getAngle();
        
        //if there is no difference, then no correction neede
        //but if there is a difference, then it sets the correction to -angle
        //sets it to the inverse because the angle control is flipped from the motors
        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.
        
        //multiplies the correction by the gain to get smooth correction
        correction = correction * gain;

        //return the finished var, corrections
        return correction;
    }
    
    private double checkDirectionB()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .05;
        
        //sets the current angle to the new globalAngle after deltaAngle has been added
        angle = getAngle();
        
        //if there is no difference, then no correction neede
        //but if there is a difference, then it sets the correction to -angle
        //sets it to the inverse because the angle control is flipped from the motors
        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = angle;        // reverse sign of angle for correction.
        
        //multiplies the correction by the gain to get smooth correction
        correction = correction * gain;

        //return the finished var, corrections
        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

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
        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backLeft.setPower(leftPower);
        backRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
