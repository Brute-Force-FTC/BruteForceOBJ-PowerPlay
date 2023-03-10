package org.firstinspires.ftc.bruteforce.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.bruteforce.utilities.BruteForceRobot;
import org.firstinspires.ftc.bruteforce.utilities.TensorFlowUtils;


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Tfod Detection for TSE", group = "Concept")
public class TensorFlowAutonomous extends LinearOpMode {
    private Blinker control_Hub;
    private Blinker expansion_Hub;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor carouselMotor;
    private CRServo continuousServo1;
    private CRServo continuousServo2;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private Gyroscope imu;
    private Servo clawServo;
    private Servo servo2;
    private Servo servo3;
    private Servo servo4;


  /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
   * the following 4 detectable objects
   *  0: Ball,
   *  1: Cube,
   *  2: Duck,
   *  3: Marker (duck location tape marker)
   *
   *  Two additional model assets are available which only contain a subset of the objects:
   *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
   *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
   */

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */


    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */

    
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        BruteForceRobot robot = new BruteForceRobot(hardwareMap);
        TensorFlowUtils utils = new TensorFlowUtils(hardwareMap);
        //utils.initVuforia();
        //utils.initTfod(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        //robot.activateTfod();

        /** Wait for the game to begin */
        sleep(3000);
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //String Duck = robot.runTfod(opModeIsActive(), telemetry);
        
        //int tse = utils.returnTSEPositionRS(opModeIsActive(), telemetry);
        //int tse = utils.returnTSEPositionRW(opModeIsActive(), telemetry);
        //int tse = utils.returnTSEPositionBS(opModeIsActive(), telemetry);
        //int tse = utils.returnTSEPositionBW(opModeIsActive(), telemetry);
        int tse = utils.returnTSEPosition(opModeIsActive(), telemetry);
        
        telemetry.addData("**** tse = ", tse);
        if (tse == 1) {
            telemetry.addLine("pink squares");
        } else if (tse == 2) {
            telemetry.addLine("purple triangles");
        } else if (tse == 3) {
            telemetry.addLine("green circles");
        } else {
           telemetry.addLine("Nothing");
        }
        telemetry.addLine("**** Outside of the loop");
        telemetry.update();
        sleep(15000);
        
        
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    
}