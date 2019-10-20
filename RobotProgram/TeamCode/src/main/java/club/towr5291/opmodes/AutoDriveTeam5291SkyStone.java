package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.HashMap;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.ReadStepFileXML;
import club.towr5291.libraries.ImageCaptureOCV;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.LibraryStateSegAuto;
import club.towr5291.libraries.LibraryVuforiaRoverRuckus;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareDriveMotors;

import static club.towr5291.functions.Constants.stepState.STATE_COMPLETE;
import static club.towr5291.functions.Constants.stepState.STATE_INIT;
import static club.towr5291.functions.Constants.stepState.STATE_RUNNING;

/*
TOWR 5291 Autonomous
Copyright (c) 2016 TOWR5291
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Written by Ian Haden/Wyatt Ashley October 2018
2019-10-17 - Ian Haden  - Converted to Sky Stone

*/
public class AutoDriveTeam5291SkyStone extends OpModeMasterLinear {

    final int LABEL_WIDTH = 200;

    //StepStates
    private Constants.stepState mintCurrentStepDelay            = STATE_COMPLETE;
    private Constants.stepState mintCurrentStateMecanumStrafe   = STATE_COMPLETE;
    private Constants.stepState mintCurrentStateDriveHeading    = STATE_COMPLETE;
    private Constants.stepState mintCurrentStatePivotTurn       = STATE_COMPLETE;

    private int mintStartPositionLeft1;                      //Left Motor 1  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionLeft2;                      //Left Motor 2  - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight1;                     //Right Motor 1 - start position of the robot in inches, starts from 0 to the end
    private int mintStartPositionRight2;                     //Right Motor 2 - start position of the robot in inches, starts from 0 to the end
    private int mintStepLeftTarget1;                         //Left Motor 1   - encoder target position
    private int mintStepLeftTarget2;                         //Left Motor 2   - encoder target position
    private int mintStepRightTarget1;                        //Right Motor 1  - encoder target position
    private int mintStepRightTarget2;                        //Right Motor 2  - encoder target position
    private double dblStepSpeedTempLeft;
    private double dblStepSpeedTempRight;
    private double mdblStepTurnL;                            //used when decoding the step, this will indicate if the robot is turning left
    private double mdblStepTurnR;                            //used when decoding the step, this will indicate if the robot is turning right
    private double mdblRobotTurnAngle;                       //used to determine angle the robot will turn
    private int mintLastEncoderDestinationLeft1;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationLeft2;             //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight1;            //used to store the encoder destination from current Step
    private int mintLastEncoderDestinationRight2;            //used to store the encoder destination from current Step
    private boolean mblnNextStepLastPos;                     //used to detect using encoders or previous calc'd position
    private int mintStepDelay;                               //used when decoding the step, this will indicate how long the delay is on ms.

    private double mdblTurnAbsoluteGyro;
    private double mdblGyrozAccumulated;
    private double mdblTankTurnGyroRequiredHeading;
    private boolean mblnDisableVisionProcessing = false;     //used when moving to disable vision to allow faster speed reading encoders.

    private double mdblStep;                                    //Step from the step file, probably not needed
    private double mdblStepTimeout;                             //Timeout value ofthe step, the step will abort if the timeout is reached
    private String mstrRobotCommand;                            //The command the robot will execute, such as move forward, turn right etc
    private double mdblStepDistance;                            //used when decoding the step, this will indicate how far the robot is to move in inches
    private double mdblStepSpeed;                               //When a move command is executed this is the speed the motors will run at
    private boolean mblnParallel;                               //used to determine if next step will run in parallel - at same time
    private boolean mblnRobotLastPos;                           //used to determine if next step will run from end of last step or from encoder position
    private double mdblRobotParm1;                              //First Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm2;                              //Second Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm3;                              //Third Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm4;                              //Fourth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm5;                              //Fifth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private double mdblRobotParm6;                              //Sixth Parameter of the command, not all commands have paramters, A*Star has parameters, where moveing does not
    private ElapsedTime mStateTime = new ElapsedTime();         // Time into current state, used for the timeout

    //States
    private int mintStepNumber;
    private int mintCurrentStep = 1;
    private Constants.stepState mintCurrentStateStep;

    //Sensors and Vision
    private BNO055IMU imu;
    private WebcamName robotWebcam;
    private ImageCaptureOCV imageCaptureOCV = new ImageCaptureOCV();
    private LibraryVuforiaRoverRuckus SkyStoneVuforia = null;//TODO Fix Rover Ruckus
    private VuforiaTrackables SkyStoneTrackables;

    private HashMap<String, Integer> mintActiveSteps = new HashMap<>();
    private HashMap<String, Integer> mintActiveStepsCopy = new HashMap<>();

    //Setup objects and robotBase
    private HardwareDriveMotors robotDrive = new HardwareDriveMotors();   // Use 5291's hardware
    private ReadStepFileXML autonomousStepsFile = new ReadStepFileXML();
    private SharedPreferences sharedPreferences;
    private robotConfig ourRobotConfig;
    private int debug = 3;

    //Logging Objects
    private FileLogger fileLogger = null;
    private ElapsedTime runtime = new ElapsedTime();
    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard() {
        return dashboard;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard.setDefaultTextView(hardwareMap);
        dashboard.clearDisplay();
        dashboard.displayPrintf(0, LABEL_WIDTH, "Text: ", "*** Robot Data ***");

        fileLogger = new FileLogger(runtime, 1, true);
        fileLogger.open();
        fileLogger.write("Time,SysMS,Thread,Event,Desc");
        fileLogger.setEventTag("runOpMode()");
        fileLogger.writeEvent("Log Started");
        runtime.reset();
        dashboard.displayPrintf(1, "***Status*** FileLogger started");

        autonomousStepsFile.ReadStepFile(sharedPreferences);

        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        ourRobotConfig = new robotConfig(sharedPreferences, autonomousStepsFile);
        debug = ourRobotConfig.getDebug();

        //need to load initial step of a delay based on user input
        autonomousStepsFile.insertSteps(ourRobotConfig.getDelay() + 1, "DELAY", 0, 0, false, false,ourRobotConfig.getDelay() * 1000, 0, 0, 0, 0, 0, 1);

        dashboard.displayPrintf(10, "initRobot IMU Loading");
        fileLogger.writeEvent(4, "initRobot -- ", "IMU Loading");

        fileLogger.writeEvent(1, "Using IMU -- " + ourRobotConfig.isEnableAdafriutIMU());
        if (ourRobotConfig.isEnableAdafriutIMU()) {
            BNO055IMU.Parameters parametersAdafruitImu = new BNO055IMU.Parameters();
            parametersAdafruitImu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parametersAdafruitImu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parametersAdafruitImu.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parametersAdafruitImu.loggingEnabled = true;
            parametersAdafruitImu.loggingTag = "IMU";
            parametersAdafruitImu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parametersAdafruitImu);
        }
        dashboard.displayPrintf(10, "initRobot IMU Finish Loading");
        fileLogger.writeEvent(4, "initRobot IMU Finish Loading");

        dashboard.displayPrintf(10, "initRobot Configuring Base");
        fileLogger.writeEvent(4, "initRobot Configuring Base");

        robotDrive.init(fileLogger, hardwareMap, ourRobotConfig.getBASE_TYPE(), ourRobotConfig.getMOTOR_KIND());
        robotDrive.setHardwareDriveResetEncoders();
        robotDrive.setHardwareDriveRunUsingEncoders();

        dashboard.displayPrintf(10, "initRobot Configuring Base Finished");
        fileLogger.writeEvent(4, "initRobot Configuring Base Finished");

        initDefaultStates();

        dashboard.displayPrintf(10, "initRobot Configuring Vuforia");
        fileLogger.writeEvent(4, "initRobot Configuring Vuforia");

        if (ourRobotConfig.isEnableVuforia() || ourRobotConfig.isEnableOpenCV()){
            //TODO make a library skystone vuforia
            SkyStoneVuforia = new LibraryVuforiaRoverRuckus();

            if (ourRobotConfig.isEnableVuforiaWebCam()){
                robotWebcam = hardwareMap.get(WebcamName.class, "Webcam");
                SkyStoneTrackables = SkyStoneVuforia.LibraryVuforiaRoverRuckus(hardwareMap, ourRobotConfig, robotWebcam, false);
            } else {
                SkyStoneTrackables = SkyStoneVuforia.LibraryVuforiaRoverRuckus(hardwareMap, ourRobotConfig, false);
            }


            dashboard.displayPrintf(10, "initRobot Configuring OpenCV");
            fileLogger.writeEvent(4, "initRobot Configuring OpenCV");

            if (ourRobotConfig.isEnableOpenCV()) {
                initOpenCv();
                imageCaptureOCV.initImageCaptureOCV(SkyStoneVuforia, dashboard, fileLogger);
            }

            dashboard.displayPrintf(10, "initRobot Configuring OpenCV Finished");
            fileLogger.writeEvent(4, "initRobot Configuring OpenCV Finished");

            SkyStoneTrackables.activate();
        }

        dashboard.displayPrintf(10, "initRobot Configuring Vuforia Finish");
        fileLogger.writeEvent(4, "initRobot Configuring Vuforia Finish");

        if (ourRobotConfig.isEnableAdafriutIMU())
            imu.stopAccelerationIntegration();

        dashboard.displayPrintf(10, "Robot is ready to start!");
        fileLogger.writeEvent(1, "Ready for start");

        waitForStart();

        dashboard.clearDisplay();
        fileLogger.setEventTag("opModeIsActive()");
        fileLogger.writeEvent(1,"Robot Running");

        if(ourRobotConfig.isEnableAdafriutIMU()) imu.startAccelerationIntegration(new Position(), new Velocity(), 500);

        while (opModeIsActive()){
            switch (mintCurrentStateStep) {
                case STATE_INIT:
                    fileLogger.writeEvent(1,"mintCurrentStateStep:- " + mintCurrentStateStep + " mintCurrentStateStep " + mintCurrentStateStep);
                    fileLogger.writeEvent(1,"About to check if step exists " + mintCurrentStep);

                    // get step from hashmap, send it to the initStep for decoding
                    if (autonomousStepsFile.activeSteps().containsKey(String.valueOf(mintCurrentStep))) {
                        fileLogger.writeEvent(1,"Step Exists TRUE " + mintCurrentStep + " about to get the values from the step");
                        initStep();
                        mintCurrentStateStep = STATE_RUNNING;
                    } else {
                        mintCurrentStateStep = Constants.stepState.STATE_FINISHED;
                    }
                    break;
                case STATE_START:
                    mintCurrentStateStep = STATE_RUNNING;
                    break;
                case STATE_RUNNING:

                    //load all the parallel steps so they can be evaluated for completeness
                    loadParallelSteps();

                    //Process all the parallel steps
                    for (String stKey : mintActiveStepsCopy.keySet()) {
                        fileLogger.writeEvent(1, "STATE_RUNNING", "Looping through Parallel steps, found " + stKey);
                        mintStepNumber = mintActiveStepsCopy.get(stKey);
                        loadActiveStep(mintStepNumber);
                        fileLogger.writeEvent(1, "STATE_RUNNING", "About to run " + mstrRobotCommand);
                        processSteps(mstrRobotCommand);
                    }

                    //Check the status of all the steps if all the states are complete we can move to the next state
                    if (checkAllStatesComplete()) {
                        mintCurrentStateStep = Constants.stepState.STATE_COMPLETE;
                    }

                    //make sure we load the current step to determine if parallel, if the steps are run out of order and a previous step was parallel
                    //things get all messed up and a step that isn't parallel can be assumed to be parallel
                    loadActiveStep(mintCurrentStep);
                    if (mblnParallel){
                        // mark this step as complete and do next one, the current step should continue to run.  Not all steps are compatible with being run in parallel
                        // like drive steps, turns etc
                        // Drive forward and shoot
                        // Drive forward and detect beacon
                        // are examples of when parallel steps should be run
                        // errors will occur if other combinations are run
                        // only go to next step if current step equals the one being processed for parallelism.
                        for (String stKey : mintActiveStepsCopy.keySet()) {
                            mintStepNumber = mintActiveStepsCopy.get(stKey);
                            if (mintCurrentStep == mintStepNumber)
                                mintCurrentStateStep = Constants.stepState.STATE_COMPLETE;
                        }
                    }
                    break;
                case STATE_PAUSE:
                    break;
                case STATE_COMPLETE:
                    fileLogger.writeEvent(1,"Step Complete - Current Step:- " + mintCurrentStep);
                    //  Transition to a new state and next step.
                    mintCurrentStep++;
                    mintCurrentStateStep = STATE_INIT;
                    break;
                case STATE_TIMEOUT:
                    robotDrive.setHardwareDrivePower(0);
                    //  Transition to a new state.
                    mintCurrentStateStep = Constants.stepState.STATE_FINISHED;
                    break;
                case STATE_ERROR:
                    dashboard.displayPrintf(1, LABEL_WIDTH,"STATE", "ERROR WAITING TO FINISH " + mintCurrentStep);
                    break;
                case STATE_FINISHED:
                    robotDrive.setHardwareDrivePower(0);

                    if (ourRobotConfig.isEnableAdafriutIMU()) imu.close();
                    //stop the logging
                    if (fileLogger != null) {
                        fileLogger.writeEvent(1, "Step FINISHED - FINISHED");
                        fileLogger.writeEvent(1, "Stopped");
                        Log.d("END:-", "FileLogger Stopped");
                        fileLogger.close();
                        fileLogger = null;
                    }
                    //deactivate vuforia
                    if (ourRobotConfig.isEnableVuforia()) SkyStoneTrackables.deactivate();
                    dashboard.displayPrintf(1, LABEL_WIDTH,"STATE", "FINISHED " + mintCurrentStep);
                    break;
            }
        }
    }

    private void loadActiveStep(int step) {
        fileLogger.setEventTag("loadActiveStep()");
        LibraryStateSegAuto mStateSegAuto = autonomousStepsFile.activeSteps().get(String.valueOf(step));
        fileLogger.writeEvent(1,"Got the values for step " + step + " about to decode");

        mdblStep            = mStateSegAuto.getmStep();
        mdblStepTimeout     = mStateSegAuto.getmRobotTimeOut();
        mstrRobotCommand    = mStateSegAuto.getmRobotCommand();
        mdblStepDistance    = mStateSegAuto.getmRobotDistance();
        mdblStepSpeed       = mStateSegAuto.getmRobotSpeed();
        mblnParallel        = mStateSegAuto.getmRobotParallel();
        mblnRobotLastPos    = mStateSegAuto.getmRobotLastPos();
        mdblRobotParm1      = mStateSegAuto.getmRobotParm1();
        mdblRobotParm2      = mStateSegAuto.getmRobotParm2();
        mdblRobotParm3      = mStateSegAuto.getmRobotParm3();
        mdblRobotParm4      = mStateSegAuto.getmRobotParm4();
        mdblRobotParm5      = mStateSegAuto.getmRobotParm5();
        mdblRobotParm6      = mStateSegAuto.getmRobotParm6();

    }

    private void loadParallelSteps() {
        fileLogger.setEventTag("loadParallelSteps()");
        mintActiveStepsCopy.clear();
        for (String stKey : mintActiveSteps.keySet()) {
            fileLogger.writeEvent(2,"Loading Active Parallel Step " + stKey);
            mintActiveStepsCopy.put(stKey, mintActiveSteps.get(stKey));
        }
    }

    private void deleteParallelStep() {
        fileLogger.setEventTag("deleteParallelStep()");
        for (String stKey : mintActiveStepsCopy.keySet()) {
            int tempStep = mintActiveStepsCopy.get(stKey);
            if (mintStepNumber == tempStep) {
                fileLogger.writeEvent(2,"Removing Parallel Step " + tempStep);
                mintActiveSteps.remove(stKey);
            }
        }
    }

    private void DelayStep(){
        fileLogger.setEventTag("DelayStep()");
        switch (mintCurrentStepDelay) {
            case STATE_INIT: {
                mintStepDelay = (int)(mdblRobotParm1);
                mintCurrentStepDelay = STATE_RUNNING;
                fileLogger.writeEvent(3,"Init Delay Time......." + mintStepDelay);
            }
            break;
            case STATE_RUNNING:
            {
                if (mStateTime.milliseconds() >= mintStepDelay)
                {
                    fileLogger.writeEvent(1,"Complete.......");
                    mintCurrentStepDelay = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout)
            {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStepDelay = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void DriveStepHeading() {
        fileLogger.setEventTag("DriveStepHeading()");

        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        double dblDistanceToEnd;
        double dblDistanceFromStartLeft1;
        double dblDistanceFromStartLeft2;
        double dblDistanceFromStartRight1;
        double dblDistanceFromStartRight2;
        double dblDistanceFromStart;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;
        double dblMaxSpeed;
        double dblError;
        double dblSteer;
        double dblLeftSpeed;
        double dblRightSpeed;

        switch (mintCurrentStateDriveHeading) {
            case STATE_INIT:

                // set motor controller to mode
                robotDrive.setHardwareDriveRunUsingEncoders();
                mblnDisableVisionProcessing = true;  //disable vision processing
                fileLogger.writeEvent(2,"mdblStepDistance   :- " + mdblStepDistance);
                // Determine new target position
                if (mblnNextStepLastPos) {
                    mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                    mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                    mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                    mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                } else {
                    mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                    mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                    mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                    mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                }
                mblnNextStepLastPos = false;

                mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());
                mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepDistance * ourRobotConfig.getCOUNTS_PER_INCH());

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                fileLogger.writeEvent(2,"mStepLeftTarget1 :- " + mintStepLeftTarget1 + " mStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(2,"mStepRightTarget1:- " + mintStepRightTarget1 + " mStepRightTarget2:- " + mintStepRightTarget2);

                // set motor controller to mode, Turn On RUN_TO_POSITION
                robotDrive.setHardwareDriveRunToPosition();

                mintCurrentStateDriveHeading = Constants.stepState.STATE_START;
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));

                dblStepSpeedTempRight = mdblStepSpeed;
                dblStepSpeedTempLeft = mdblStepSpeed;
                break;

            case STATE_START:
                if (robotDrive.getHardwareBaseDriveBusy()) {
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_RUNNING;
                    fileLogger.writeEvent(2,"Base Drive Not Running Yet ");
                }
                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));
                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {// Stop all motion;
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1,"Timeout Drive didn't engage:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                }
                break;

            case STATE_RUNNING:
                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                robotDrive.setHardwareDrivePower(Math.abs(mdblStepSpeed));

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                // ramp up speed - need to write function to ramp up speed
                dblDistanceFromStartLeft1 = Math.abs(mintStartPositionLeft1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceFromStartLeft2 = Math.abs(mintStartPositionLeft2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceFromStartRight1 = Math.abs(mintStartPositionRight1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceFromStartRight2 = Math.abs(mintStartPositionRight2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                //if moving ramp up
                dblDistanceFromStart = (dblDistanceFromStartLeft1 + dblDistanceFromStartRight1 + dblDistanceFromStartLeft2 + dblDistanceFromStartRight2) / 4;

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                //if getting close ramp down speed
                dblDistanceToEnd = Math.max(Math.max(Math.max(Math.abs(dblDistanceToEndLeft1),Math.abs(dblDistanceToEndRight1)),Math.abs(dblDistanceToEndLeft2)),Math.abs(dblDistanceToEndRight2));

                //parameter 1 or 4 is use gyro for direction,  setting either of these to 1 will get gyro correction
                // if parameter 1 is true
                // parameter 2 is the error
                // parameter 3 is the gain coefficient
                if ((mdblRobotParm1 == 1) || (mdblRobotParm4 == 1)) {
                    //use Gyro to run heading
                    // adjust relative speed based on heading error.
                    dblError = getDriveError(mdblRobotParm1);
                    dblSteer = getDriveSteer(dblError, mdblRobotParm1);
                    fileLogger.writeEvent(3,"dblError " + dblError);
                    fileLogger.writeEvent(3,"dblSteer " + dblSteer);
                    fileLogger.writeEvent(3, "runningDriveHeadingStep", "Heading " + mdblRobotParm2);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (mdblStepDistance < 0)
                        dblSteer *= -1.0;

                    dblStepSpeedTempLeft = dblStepSpeedTempLeft - dblSteer;
                    dblStepSpeedTempRight = dblStepSpeedTempRight + dblSteer;

                    // Normalize speeds if any one exceeds +/- 1.0;
                    dblMaxSpeed = Math.max(Math.abs(dblStepSpeedTempLeft), Math.abs(dblStepSpeedTempRight));
                    if (dblMaxSpeed > 1.0) {
                        dblStepSpeedTempLeft /= dblMaxSpeed;
                        dblStepSpeedTempRight /= dblMaxSpeed;
                    }
                }
                fileLogger.writeEvent(3,"dblDistanceToEnd " + dblDistanceToEnd);

                if (mblnRobotLastPos) {
                    if (Math.abs(dblDistanceToEnd) <= mdblRobotParm6) {
                        mblnNextStepLastPos = true;
                        fileLogger.writeEvent(3,"mblnRobotLastPos Complete Near END " + Math.abs(dblDistanceToEnd));
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                } else if (Math.abs(dblDistanceToEnd) <= mdblRobotParm6) {
                    fileLogger.writeEvent(3,"mblnRobotLastPos Complete Near END " + Math.abs(dblDistanceToEnd));
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                //stop driving when within .25 inch, sometimes takes a long time to get that last bit and times out.
                //stop when drive motors stop
                if (Math.abs(dblDistanceToEnd) <= 0.25) {
                    fileLogger.writeEvent(3,"mblnRobotLastPos Complete Near END " + Math.abs(dblDistanceToEnd));
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                } else if (robotDrive.getHardwareBaseDriveBusy()) {
                    fileLogger.writeEvent(3,"Encoder counts per inch = " + ourRobotConfig.getCOUNTS_PER_INCH() + " dblDistanceFromStart " + dblDistanceFromStart + " dblDistanceToEnd " + dblDistanceToEnd + " Power Level Left " + dblStepSpeedTempLeft + " Power Level Right " + dblStepSpeedTempRight + " Running to target  L1, L2, R1, R2  " + mintStepLeftTarget1 + ", " + mintStepLeftTarget2 + ", " + mintStepRightTarget1 + ",  " + mintStepRightTarget2 + ", " + " Running at position L1 " + intLeft1MotorEncoderPosition + " L2 " + intLeft2MotorEncoderPosition + " R1 " + intRight1MotorEncoderPosition + " R2 " + intRight2MotorEncoderPosition);
                    dashboard.displayPrintf(3, LABEL_WIDTH,"Path1", "Running to " + mintStepLeftTarget1 + ":" + mintStepRightTarget1);
                    dashboard.displayPrintf(4, LABEL_WIDTH,"Path2", "Running at " +intLeft1MotorEncoderPosition + ":" + intRight1MotorEncoderPosition);
                    dashboard.displayPrintf(5, LABEL_WIDTH,"Path3", "Running at " + intLeft2MotorEncoderPosition + ":" + intRight2MotorEncoderPosition);
                    // set power on motor controller to update speeds
                    robotDrive.setHardwareDriveLeftMotorPower(dblStepSpeedTempLeft);
                    robotDrive.setHardwareDriveRightMotorPower(dblStepSpeedTempRight);
                } else {
                    //robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(2,"Complete         ");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                //check timeout value
                if (mStateTime.seconds() > mdblStepTimeout) {// Stop all motion;
                    robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                    //  Transition to a new state.
                    mintCurrentStateDriveHeading = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }
                break;
        }
    }

    private double getDriveError(double targetAngle) {
        fileLogger.setEventTag("getDriveError()");
        double robotError;
        double robotErrorIMU;
        double robotErrorGyro;
        double MRgyroHeading;
        double adafruitIMUHeading;

        adafruitIMUHeading = getAdafruitHeading();

        fileLogger.writeEvent(2,"targetAngle " + targetAngle);
        fileLogger.writeEvent(2,"Adafruit IMU Reading " + adafruitIMUHeading);
        // calculate error in -179 to +180 range  (
        robotErrorIMU = targetAngle - teamAngleAdjust(adafruitIMUHeading);
        robotError = robotErrorIMU;
        fileLogger.writeEvent(2,"USING HEADING FROM IMU=" + ourRobotConfig.isEnableAdafriutIMU());
        fileLogger.writeEvent(2,"robotErrorIMU " + robotError + ", getAdafruitHeading() " + adafruitIMUHeading + " teamAngleAdjust(adafruitIMUHeading) "  + teamAngleAdjust(adafruitIMUHeading));

        if (robotError > 180)
            robotError -= 360;
        if (robotError <= -180)
            robotError += 360;

        fileLogger.writeEvent(2,"robotError2 " + robotError);
        return -robotError;
    }

    private void PivotTurnStep()  //should be same as radius turn with radius of 1/2 robot width, so this function can be deleted once radius turn is completed
    {
        fileLogger.setEventTag("PivotTurnStep()");

        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;

        switch (mintCurrentStatePivotTurn) {
            case STATE_INIT: {
                // set motor controller to mode
                robotDrive.setHardwareDriveRunUsingEncoders();
                mblnDisableVisionProcessing = true;  //disable vision processing
                mdblStepTurnL = 0;
                mdblStepTurnR = 0;

                if (mdblStepDistance > 0) {
                    mdblStepTurnL = mdblStepDistance;
                    mdblStepTurnR = 0;
                } else {
                    mdblStepTurnL = 0;
                    mdblStepTurnR = mdblStepDistance;
                }

                fileLogger.writeEvent(2,"mdblStepTurnL      :- " + mdblStepTurnL);
                fileLogger.writeEvent(2,"mdblStepTurnR      :- " + mdblStepTurnR);

                // Turn On RUN_TO_POSITION
                if (mdblStepTurnR == 0) {
                    // Determine new target position
                    fileLogger.writeEvent(2,"Current LPosition:-" + robotDrive.baseMotor1.getCurrentPosition());

                    // Get Current Encoder positions
                    if (mblnNextStepLastPos) {
                        mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                        mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                        mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                        mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                    } else {
                        mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                        mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                        mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                        mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                    }
                    mblnNextStepLastPos = false;

                    // Determine new target position
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepTurnL * ourRobotConfig.getDblCountsPerDegree());
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepTurnL * ourRobotConfig.getDblCountsPerDegree());
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepTurnR * ourRobotConfig.getDblCountsPerDegree());
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepTurnR * ourRobotConfig.getDblCountsPerDegree());

                    //store the encoder positions so next step can calculate destination
                    mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                    mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                    mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                    mintLastEncoderDestinationRight2 = mintStepRightTarget2;
                    fileLogger.writeEvent(2,"mintStepLeftTarget1:-  " + mintStepLeftTarget1 + " mintStepLeftTarget2:-  " + mintStepLeftTarget2);

                    // pass target position to motor controller
                    robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                    robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);

                    // set left motor controller to mode
                    robotDrive.setHardwareDriveLeftRunToPosition();

                    // set power on motor controller to start moving
                    robotDrive.setHardwareDriveLeftMotorPower(Math.abs(mdblStepSpeed));
                } else {
                    // Determine new target position
                    fileLogger.writeEvent(2,"Current RPosition:-" + robotDrive.baseMotor3.getCurrentPosition());

                    // Get Current Encoder positions
                    if (mblnNextStepLastPos) {
                        mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                        mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                        mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                        mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                    } else {
                        mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                        mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                        mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                        mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                    }
                    mblnNextStepLastPos = false;

                    // Determine new target position
                    mintStepLeftTarget1 = mintStartPositionLeft1 + (int) (mdblStepTurnL * ourRobotConfig.getDblCountsPerDegree());
                    mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepTurnL * ourRobotConfig.getDblCountsPerDegree());
                    mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepTurnR * ourRobotConfig.getDblCountsPerDegree());
                    mintStepRightTarget2 = mintStartPositionRight2 + (int) (mdblStepTurnR * ourRobotConfig.getDblCountsPerDegree());

                    //store the encoder positions so next step can calculate destination
                    mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                    mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                    mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                    mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                    fileLogger.writeEvent(3,"mintStepRightTarget1:- " + mintStepRightTarget1 + " mintStepRightTarget2:- " + mintStepRightTarget2);

                    // pass target position to motor controller
                    robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                    robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                    // set right motor controller to mode, Turn On RUN_TO_POSITION
                    robotDrive.setHardwareDriveRightRunToPosition();

                    // set power on motor controller to start moving
                    robotDrive.setHardwareDriveRightMotorPower(Math.abs(mdblStepSpeed));
                }

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                fileLogger.writeEvent(3,"gblStepLeftTarget:- " + mintStepLeftTarget1 + " mintStepLeftTarget2 :- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"gblStepRightTarget:- " + mintStepRightTarget1 + " mintStepRightTarget2:- " + mintStepRightTarget2);
                mintCurrentStatePivotTurn = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();

                fileLogger.writeEvent(3,"Current LPosition1:-" + robotDrive.baseMotor1.getCurrentPosition() + " LTarget:- " + mintStepLeftTarget1 + " LPosition2:-" + robotDrive.baseMotor2.getCurrentPosition() + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3,"Current RPosition1:-" + robotDrive.baseMotor3.getCurrentPosition() + " RTarget:- " + mintStepRightTarget1 + " RPosition2:-" + robotDrive.baseMotor4.getCurrentPosition() + " RTarget2:- " + mintStepRightTarget2);

                if (mdblStepTurnR == 0) {
                    fileLogger.writeEvent(3,"Running.......");
                    dashboard.displayPrintf(3, LABEL_WIDTH, "Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, LABEL_WIDTH,"Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, LABEL_WIDTH,"ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                    if (mblnRobotLastPos) {
                        if (((dblDistanceToEndLeft1 + dblDistanceToEndLeft2) / 2) < 2.0) {
                            mblnNextStepLastPos = true;
                            mblnDisableVisionProcessing = false;  //enable vision processing
                            mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                            deleteParallelStep();
                        }
                    }
                    //if (!robotDrive.leftMotor1.isBusy()) {
                    //get motor busy state bitmap is right2, right1, left2, left1
                    if (((robotDrive.getHardwareDriveIsBusy() & robotConfig.motors.leftMotor1.toInt()) == robotConfig.motors.leftMotor1.toInt())) {
                        fileLogger.writeEvent(1,"Complete........");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                    }
                } else if (mdblStepTurnL == 0) {
                    fileLogger.writeEvent(3,"Running.......");
                    dashboard.displayPrintf(3, LABEL_WIDTH,"Target", "Running to %7d :%7d", mintStepLeftTarget1, mintStepRightTarget1);
                    dashboard.displayPrintf(4, LABEL_WIDTH,"Actual_Left", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                    dashboard.displayPrintf(5, LABEL_WIDTH,"ActualRight", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                    if (mblnRobotLastPos) {
                        if (((dblDistanceToEndRight1 + dblDistanceToEndRight2) / 2) < 2.2) {
                            mblnNextStepLastPos = true;
                            mblnDisableVisionProcessing = false;  //enable vision processing
                            mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                            deleteParallelStep();
                            break;
                        }
                    }

                    //get motor busy state bitmap is right2, right1, left2, left1
                    if (!robotDrive.getHardwareBaseDriveBusy()) {
                        fileLogger.writeEvent(1,"Complete.......");
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                } else {
                    // Stop all motion by setting power to 0
                    //robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1,"Complete.......");
                    mblnDisableVisionProcessing = false;  //enable vision processing
                    mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }
            }
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1,"Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStatePivotTurn = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
            }
            break;
        }
    }

    private void MecanumStrafe() {
        fileLogger.setEventTag("MecanumStrafe()");

        double dblDistanceToEndLeft1;
        double dblDistanceToEndLeft2;
        double dblDistanceToEndRight1;
        double dblDistanceToEndRight2;
        int intLeft1MotorEncoderPosition;
        int intLeft2MotorEncoderPosition;
        int intRight1MotorEncoderPosition;
        int intRight2MotorEncoderPosition;
        double rdblSpeed;

        switch (mintCurrentStateMecanumStrafe) {
            case STATE_INIT: {
                double adafruitIMUHeading;
                double currentHeading;

                robotDrive.setHardwareDriveRunUsingEncoders();

                if (ourRobotConfig.isEnableAdafriutIMU()) {
                    adafruitIMUHeading = getAdafruitHeading();
                    currentHeading = adafruitIMUHeading;
                    //mdblRobotTurnAngle = Double.parseDouble(mdblStepDistance);
                    //fileLogger.writeEvent(3, "MecanumStrafe", "USING HEADING FROM IMU=" + useAdafruitIMU);
                    //fileLogger.writeEvent(3, "MecanumStrafe()", "mdblRobotTurnAngle " + mdblRobotTurnAngle + " currentHeading " + currentHeading);
                    //mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirection((int) currentHeading, (int) mdblRobotTurnAngle).substring(3));
                }

                robotDrive.setHardwareDriveRunToPosition();

                // Get Current Encoder positions
                if (mblnNextStepLastPos) {
                    mintStartPositionLeft1 = mintLastEncoderDestinationLeft1;
                    mintStartPositionLeft2 = mintLastEncoderDestinationLeft2;
                    mintStartPositionRight1 = mintLastEncoderDestinationRight1;
                    mintStartPositionRight2 = mintLastEncoderDestinationRight2;
                } else {
                    mintStartPositionLeft1 = robotDrive.baseMotor1.getCurrentPosition();
                    mintStartPositionLeft2 = robotDrive.baseMotor2.getCurrentPosition();
                    mintStartPositionRight1 = robotDrive.baseMotor3.getCurrentPosition();
                    mintStartPositionRight2 = robotDrive.baseMotor4.getCurrentPosition();
                }
                mblnNextStepLastPos = false;

                mintStepLeftTarget1 = mintStartPositionLeft1 - (int) (mdblStepDistance * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET());
                mintStepLeftTarget2 = mintStartPositionLeft2 + (int) (mdblStepDistance * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET());
                mintStepRightTarget1 = mintStartPositionRight1 + (int) (mdblStepDistance * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET());
                mintStepRightTarget2 = mintStartPositionRight2 - (int) (mdblStepDistance * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE() * ourRobotConfig.getDblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET());

                //store the encoder positions so next step can calculate destination
                mintLastEncoderDestinationLeft1 = mintStepLeftTarget1;
                mintLastEncoderDestinationLeft2 = mintStepLeftTarget2;
                mintLastEncoderDestinationRight1 = mintStepRightTarget1;
                mintLastEncoderDestinationRight2 = mintStepRightTarget2;

                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);

                // set motor controller to mode
                robotDrive.setHardwareDriveRunToPosition();
                rdblSpeed = mdblStepSpeed;

                if (rdblSpeed >= 0.7) {
                    rdblSpeed = 0.7;  //This is the maximum speed, anything above 0.6 is the same as a speed of 1 for drive to position
                }
                // set power on motor controller to start moving
                robotDrive.setHardwareDrivePower(rdblSpeed);  //set motor power
                mintCurrentStateMecanumStrafe = Constants.stepState.STATE_RUNNING;
            }
            break;
            case STATE_RUNNING: {
                rdblSpeed = mdblStepSpeed;
                if (rdblSpeed >= 0.7) {
                    rdblSpeed = 0.7;  //This is the maximum speed, anything above 0.6 is the same as a speed of 1 for drive to position
                }
                // pass target position to motor controller
                robotDrive.baseMotor1.setTargetPosition(mintStepLeftTarget1);
                robotDrive.baseMotor2.setTargetPosition(mintStepLeftTarget2);
                robotDrive.baseMotor3.setTargetPosition(mintStepRightTarget1);
                robotDrive.baseMotor4.setTargetPosition(mintStepRightTarget2);
                robotDrive.setHardwareDrivePower(rdblSpeed);  //set motor power

                if(ourRobotConfig.isEnableAdafriutIMU()) {
                    double adafruitIMUHeading;

                    adafruitIMUHeading = getAdafruitHeading();
                    mdblGyrozAccumulated = adafruitIMUHeading;
                    mdblGyrozAccumulated = teamAngleAdjust(mdblGyrozAccumulated);//Set variables to MRgyro readings
                    //mdblTurnAbsoluteGyro = Double.parseDouble(newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(3));
                    String mstrDirection = (newAngleDirectionGyro((int) mdblGyrozAccumulated, (int) mdblRobotTurnAngle).substring(0, 3));
                    fileLogger.writeEvent(3, "USING HEADING FROM IMU=" + ourRobotConfig.isEnableAdafriutIMU());
                    fileLogger.writeEvent(3, "Running, mdblGyrozAccumulated = " + mdblGyrozAccumulated);
                    fileLogger.writeEvent(3, "Running, mdblTurnAbsoluteGyro = " + mdblTurnAbsoluteGyro);
                    fileLogger.writeEvent(3, "Running, mstrDirection        = " + mstrDirection);
                    fileLogger.writeEvent(3, "Running, adafruitIMUHeading   = " + adafruitIMUHeading);
                }

                intLeft1MotorEncoderPosition = robotDrive.baseMotor1.getCurrentPosition();
                intLeft2MotorEncoderPosition = robotDrive.baseMotor2.getCurrentPosition();
                intRight1MotorEncoderPosition = robotDrive.baseMotor3.getCurrentPosition();
                intRight2MotorEncoderPosition = robotDrive.baseMotor4.getCurrentPosition();

                //determine how close to target we are
                dblDistanceToEndLeft1 = (mintStepLeftTarget1 - intLeft1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndLeft2 = (mintStepLeftTarget2 - intLeft2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight1 = (mintStepRightTarget1 - intRight1MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                dblDistanceToEndRight2 = (mintStepRightTarget2 - intRight2MotorEncoderPosition) / ourRobotConfig.getCOUNTS_PER_INCH();
                fileLogger.writeEvent(3, "Current LPosition1:- " + intLeft1MotorEncoderPosition + " LTarget1:- " + mintStepLeftTarget1);
                fileLogger.writeEvent(3, "Current LPosition2:- " + intLeft2MotorEncoderPosition + " LTarget2:- " + mintStepLeftTarget2);
                fileLogger.writeEvent(3, "Current RPosition1:- " + intRight1MotorEncoderPosition + " RTarget1:- " + mintStepRightTarget1);
                fileLogger.writeEvent(3, "Current RPosition2:- " + intRight2MotorEncoderPosition + " RTarget2:- " + mintStepRightTarget2);
                dashboard.displayPrintf(4,  "Mecanum Strafe Positions moving " + mdblStepDistance);
                dashboard.displayPrintf(5, LABEL_WIDTH, "Left  Target: ", "Running to %7d :%7d", mintStepLeftTarget1, mintStepLeftTarget2);
                dashboard.displayPrintf(6, LABEL_WIDTH, "Left  Actual: ", "Running at %7d :%7d", intLeft1MotorEncoderPosition, intLeft2MotorEncoderPosition);
                dashboard.displayPrintf(7, LABEL_WIDTH, "Right Target: ", "Running to %7d :%7d", mintStepRightTarget1, mintStepRightTarget2);
                dashboard.displayPrintf(8, LABEL_WIDTH, "Right Actual: ", "Running at %7d :%7d", intRight1MotorEncoderPosition, intRight2MotorEncoderPosition);

                double dblDistanceToEnd = Math.max(Math.max(Math.max(Math.abs(dblDistanceToEndLeft1),Math.abs(dblDistanceToEndRight1)),Math.abs(dblDistanceToEndLeft2)),Math.abs(dblDistanceToEndRight2));

                if (mblnRobotLastPos) {
                    if (Math.abs(dblDistanceToEnd) <= mdblRobotParm6) {
                        fileLogger.writeEvent(3,"Complete NextStepLasp......." + dblDistanceToEnd);
                        mblnNextStepLastPos = true;
                        mblnDisableVisionProcessing = false;  //enable vision processing
                        mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                        deleteParallelStep();
                        break;
                    }
                } else if (Math.abs(dblDistanceToEnd) <= mdblRobotParm6) {
                    fileLogger.writeEvent(3,"Complete Early ......." + dblDistanceToEnd);
                    fileLogger.writeEvent(3,"mblnRobotLastPos Complete Near END ");
                    mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

                //stop driving when within .25 inch, sometimes takes a long time to get that last bit and times out.
                //stop when drive motors stop
                if (Math.abs(dblDistanceToEnd) <= 0.25) {
                    fileLogger.writeEvent(3,"mblnRobotLastPos Complete Near END " + Math.abs(dblDistanceToEnd));
                    mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                } else if (!robotDrive.getHardwareBaseDriveBusy()) {
                    //robotDrive.setHardwareDrivePower(0);
                    fileLogger.writeEvent(1, "Complete         ");
                    mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                    deleteParallelStep();
                    break;
                }

            } //end Case Running
            //check timeout value
            if (mStateTime.seconds() > mdblStepTimeout) {
                fileLogger.writeEvent(1, "Timeout:- " + mStateTime.seconds());
                //  Transition to a new state.
                mintCurrentStateMecanumStrafe = Constants.stepState.STATE_COMPLETE;
                deleteParallelStep();
                break;
            }
            break;
        }
    }

    private Double getAdafruitHeading()
    {
        Orientation angles;
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return angleToHeading(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    //for adafruit IMU
    private Double formatAngle(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.fromUnit(angleUnit, angle);
    }

    private double teamAngleAdjust ( double angle ) {
        fileLogger.setEventTag("teamAngleAdjust()");
        fileLogger.writeEvent(2,"teamAngleAdjust - angle " + angle + " allianceColor " + ourRobotConfig.getAllianceColor());

        if (ourRobotConfig.getAllianceColor().equals("Red")) {
            //angle = angle + 90;  if starting against the wall
            //angle = angle + 225; if starting at 45 to the wall facing the beacon
            angle = angle + 225;
            if (angle > 360) {
                angle = angle - 360;
            }
            fileLogger.writeEvent(2,"In RED Angle " + angle);

        } else
        if (ourRobotConfig.getAllianceColor().equals("Blue")) {
            //angle = angle - 180;;  if starting against the wall
            angle = angle - 135;
            if (angle < 0) {
                angle = angle + 360;
            }
        }
        return angle;
    }

    private double getDriveSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //for adafruit IMU as it returns z angle only
    private double angleToHeading(double z) {
        double angle = -z;// + imuStartCorrectionVar + imuMountCorrectionVar;
        if (angle < 0)
            return angle + 360;
        else if (angle > 360)
            return angle - 360;
        else
            return angle;
    }

    private String newAngleDirectionGyro (int currentDirection, int newDirection){
        fileLogger.setEventTag("newAngleDirectionGyro()");
        int intAngle1;

        //calculate the smallest angle
        if (currentDirection < newDirection) {
            intAngle1 = (newDirection - currentDirection);
            if (intAngle1 > 180)
            {
                intAngle1 = (currentDirection + 360 - newDirection);
                return "-" + intAngle1;
            }
            return "+" + intAngle1;
        }
        else
        {
            intAngle1 = (currentDirection - newDirection);
            if (intAngle1 > 180)
            {
                intAngle1 = (newDirection + 360 - currentDirection);
                return "+" + intAngle1;
            }
            return "-" + intAngle1;
        }
    }

    private void processSteps(String stepName) {
        fileLogger.setEventTag("processSteps()");
        fileLogger.writeEvent(2,"Processing Parallel Step " + stepName);
        switch (stepName) {
            case "DELAY":
                DelayStep();
                break;
            case "DRIVE":
                DriveStepHeading();
                break;
            case "LPE":
            case "RPE":
                PivotTurnStep();
                break;
            case "STRAFE":
                MecanumStrafe();
                break;
        }
    }

    private void initStep() {
        fileLogger.setEventTag("initStep()");
        fileLogger.writeEvent(3,"Starting to Decode Step " + mintCurrentStep);

        if (!(mintActiveSteps.containsValue(mintCurrentStep))) {
            mintActiveSteps.put(String.valueOf(mintCurrentStep), mintCurrentStep);
            fileLogger.writeEvent(3,"Put step into hashmap mintActiveSteps " + mintCurrentStep);
        }

        loadActiveStep(mintCurrentStep);
        // Reset the state time, and then change to next state.
        mStateTime.reset();

        switch (mstrRobotCommand) {
            case "DELAY":
                mintCurrentStepDelay                = STATE_INIT;
                break;
            case "DRIVE":
                mintCurrentStateDriveHeading        = STATE_INIT;
                break;
            case "LPE":
            case "RPE":
                mintCurrentStatePivotTurn           = STATE_INIT;
            case "STRAFE":
                mintCurrentStateMecanumStrafe       = STATE_INIT;
                break;
        }

        fileLogger.writeEvent(2,"Current Step          :- " + mintCurrentStep);
        fileLogger.writeEvent(2, "initStep()", "Current Step          :- " + mintCurrentStep);
        fileLogger.writeEvent(2, "initStep()", "mdblStepTimeout       :- " + mdblStepTimeout);
        fileLogger.writeEvent(2, "initStep()", "mdblStepSpeed         :- " + mdblStepSpeed);
        fileLogger.writeEvent(2, "initStep()", "mstrRobotCommand      :- " + mstrRobotCommand);
        fileLogger.writeEvent(2, "initStep()", "mblnParallel          :- " + mblnParallel);
        fileLogger.writeEvent(2, "initStep()", "mblnRobotLastPos      :- " + mblnRobotLastPos);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm1        :- " + mdblRobotParm1);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm2        :- " + mdblRobotParm2);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm3        :- " + mdblRobotParm3);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm4        :- " + mdblRobotParm4);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm5        :- " + mdblRobotParm5);
        fileLogger.writeEvent(2, "initStep()", "mdblRobotParm6        :- " + mdblRobotParm6);
        fileLogger.writeEvent(2, "initStep()", "mdblStepDistance      :- " + mdblStepDistance);
        fileLogger.writeEvent(2, "initStep()", "mdblStepTurnL         :- " + mdblStepTurnL);
        fileLogger.writeEvent(2, "initStep()", "mdblStepTurnR         :- " + mdblStepTurnR);
    }

    private boolean checkAllStatesComplete () {
        return (mintCurrentStepDelay == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateMecanumStrafe == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStateDriveHeading == Constants.stepState.STATE_COMPLETE) &&
                (mintCurrentStatePivotTurn == Constants.stepState.STATE_COMPLETE);
    }

    private void initDefaultStates() {
        mintCurrentStateStep                = STATE_INIT;
        mintCurrentStepDelay                = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateMecanumStrafe       = Constants.stepState.STATE_COMPLETE;
        mintCurrentStateDriveHeading        = Constants.stepState.STATE_COMPLETE;
        mintCurrentStatePivotTurn           = Constants.stepState.STATE_COMPLETE;
    }

}