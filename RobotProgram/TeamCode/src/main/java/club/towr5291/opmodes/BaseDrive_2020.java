package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.R;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareArmMotorsSkyStone;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

import static club.towr5291.functions.Constants.stepState.STATE_COMPLETE;

/*
    made by Wyatt Ashley on 8/2/2018
    Xavier was here
*/
@TeleOp(name = "Base Drive 2020", group = "5291")
public class BaseDrive_2020 extends OpModeMasterLinear {
    private Constants.stepState stepState = Constants.stepState.STATE_COMPLETE;
    boolean hold = false;

    /* Hardware Set Up */
    private HardwareDriveMotors Robot               = new HardwareDriveMotors();
    private HardwareArmMotorsSkyStone Arms          = new HardwareArmMotorsSkyStone();

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;

    private FileLogger fileLogger;
    final String TAG = "TeleOp";
    private ElapsedTime runtime                     = new ElapsedTime();
    private robotConfig ourRobotConfig;

    public double HOLDINGTILTMOTORPOWER = .2;
    private int debug;

    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = TOWRDashBoard.createInstance(telemetry);
        FtcRobotControllerActivity act = (FtcRobotControllerActivity)(hardwareMap.appContext);

        dashboard.setTextView((TextView)act.findViewById(R.id.textOpMode));
        dashboard.displayPrintf(0, "Starting Menu System");

        ourRobotConfig = new robotConfig();
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);

        ourRobotConfig.setAllianceColor(sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red"));// Using a Function to Store The Robot Specification
        ourRobotConfig.setTeamNumber(sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "0000"));
        ourRobotConfig.setAllianceStartPosition(sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left"));
        ourRobotConfig.setDelay(Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Delay", "0")));
        ourRobotConfig.setRobotMotorType(sharedPreferences.getString("club.towr5291.Autonomous.RobotMotorChoice", "REV20ORBIT"));
        ourRobotConfig.setRobotConfigBase(sharedPreferences.getString("club.towr5291.Autonomous.RobotConfigBase", "TileRunner2x40"));
        debug = Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1"));

        //now we have loaded the config from sharedpreferences we can setup the robot
        ourRobotConfig.initConfig();
        dashboard.displayPrintf(0, "Robot Config Loaded");

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        // All The Specification of the robot and controller
        fileLogger.writeEvent(1,"Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1,"Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1,"Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent(1,"Robot Base Config", ourRobotConfig.getRobotConfigBase());
        fileLogger.writeEvent(1, "Robot Motor Type", ourRobotConfig.getRobotMotorType());
        fileLogger.writeEvent(1,"Team Number", ourRobotConfig.getTeamNumber());

        Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.valueOf(ourRobotConfig.getRobotConfigBase()), LibraryMotorType.MotorTypes.valueOf(ourRobotConfig.getRobotMotorType()));// Starting robot Hardware map
        dashboard.displayPrintf(0, "Robot Base Loaded");

        Robot.allMotorsStop();

        Arms.init(hardwareMap, dashboard);
        fileLogger.writeEvent(1,"","Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        dashboard.clearDisplay();

        fileLogger.writeEvent("Starting Loop");

        //dashboard.clearDisplay();

        Arms.liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard.displayPrintf(3, "Controller A Options");
        dashboard.displayPrintf(4, "--------------------");
        dashboard.displayPrintf(8, "Controller B Options");
        dashboard.displayPrintf(9, "--------------------");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            fileLogger.writeEvent(1, "In Main Loop");

            dashboard.displayPrintf(5, "Controller Mode -- ", "Mecanum Drive Relic Recovery (BAD)");
            fileLogger.writeEvent(debug, "Controller Mode", "Mecanum Drive Relic Recovery");

            Robot.baseMotor1.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
            Robot.baseMotor2.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
            Robot.baseMotor3.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
            Robot.baseMotor4.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));

            liftMotorPower();
            Arms.intakeMotor1.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            // grab the block
            if (gamepad2.a)
                Arms.grabServo.setPosition(0);
            else if (gamepad2.y)
                Arms.grabServo.setPosition(.5);

            //rotate the arm out so we can stack the block or bring it back in
            if (gamepad2.b)
                Arms.wristServo.setPosition(0);
            else if (gamepad2.x)
                Arms.wristServo.setPosition(1);

            //send the tape measure out
            if (gamepad2.dpad_up)
                Arms.tapeServo.setPosition(.9);
            else if (gamepad2.dpad_down)
                Arms.tapeServo.setPosition(.1);
            else
                Arms.tapeServo.setPosition(0);
            //Foundation Arm

            if (gamepad2.left_bumper)
                Arms.foundationServo.setPosition(0);
            else if (gamepad2.right_bumper)
                Arms.foundationServo.setPosition(1);

        }
        //stop the logging
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "TeleOP FINISHED - FINISHED");
            fileLogger.writeEvent(1, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    } //RunOpMode

    public void liftMotorPower(){

        if ((gamepad2.left_stick_y == 0)) {
            if ((hold == false)) {
                fileLogger.writeEvent(8, "Hold is Now true");
                fileLogger.writeEvent(8,"Run using encoders now in hold function in the movement of the arm");
                Arms.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                fileLogger.writeEvent(8, "Setting the target position so that the arm does not move!");
                Arms.liftMotor1.setTargetPosition(Arms.liftMotor1.getCurrentPosition());
                hold = true;
            }
            Arms.liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Arms.liftMotor1.setPower(HOLDINGTILTMOTORPOWER);

            fileLogger.writeEvent(8, "Setting the power to the tilt motor at " + Arms.liftMotor1.getPower());
        } else {
            Arms.liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hold = false;
            Arms.liftMotor1.setPower(-gamepad2.left_stick_y);
        }
    }
}