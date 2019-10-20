package club.towr5291.opmodes;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import club.towr5291.functions.Constants;
import club.towr5291.functions.FileLogger;
import club.towr5291.functions.ReadStepFileXML;
import club.towr5291.functions.TOWR5291Tick;
import club.towr5291.functions.TOWR5291Toggle;
import club.towr5291.libraries.LibraryMotorType;
import club.towr5291.libraries.TOWR5291LEDControl;
import club.towr5291.libraries.TOWRDashBoard;
import club.towr5291.libraries.robotConfig;
import club.towr5291.libraries.robotConfigSettings;
import club.towr5291.robotconfig.HardwareArmMotorsRoverRuckus;
import club.towr5291.robotconfig.HardwareDriveMotors;
import club.towr5291.robotconfig.HardwareSensorsRoverRuckus;

import static club.towr5291.functions.Constants.stepState.STATE_COMPLETE;

/*
    made by Wyatt Ashley on 8/2/2018
*/
@TeleOp(name = "Base Drive 2019", group = "5291")
public class BaseDrive_2019 extends OpModeMasterLinear {

    /* Hardware Set Up */
    private HardwareDriveMotors Robot               = new HardwareDriveMotors();

    //Settings from the sharepreferences
    private SharedPreferences sharedPreferences;

    private FileLogger fileLogger;
    final String TAG = "RR TeleOp";
    private ElapsedTime runtime                     = new ElapsedTime();
    private club.towr5291.libraries.robotConfig ourRobotConfig;
    private ReadStepFileXML readStepFileXML;

    private int debug;

    private static TOWRDashBoard dashboard = null;
    public static TOWRDashBoard getDashboard()
    {
        return dashboard;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = TOWRDashBoard.createInstance(telemetry);
        dashboard.setDefaultTextView(hardwareMap);
        dashboard.displayPrintf(0, "Starting Menu System");

        readStepFileXML = new ReadStepFileXML();
        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
        readStepFileXML.ReadStepFile(sharedPreferences);
        ourRobotConfig = new robotConfig(sharedPreferences, readStepFileXML);

        dashboard.displayPrintf(0, "Robot Config Loaded");

        fileLogger = new FileLogger(runtime, Integer.parseInt(sharedPreferences.getString("club.towr5291.Autonomous.Debug", "1")), true);// initializing FileLogger
        fileLogger.open();// Opening FileLogger
        fileLogger.writeEvent(TAG, "Log Started");// First Line Add To Log

        // All The Specification of the robot and controller
        fileLogger.writeEvent(1,"Alliance Color", ourRobotConfig.getAllianceColor());
        fileLogger.writeEvent(1,"Alliance Start Position", ourRobotConfig.getAllianceStartPosition());
        fileLogger.writeEvent(1,"Delay", String.valueOf(ourRobotConfig.getDelay()));
        fileLogger.writeEvent(1,"Team Number", ourRobotConfig.getTeamNumber());

        Robot.init(fileLogger, hardwareMap, ourRobotConfig.getBASE_TYPE(), ourRobotConfig.getMOTOR_KIND());// Starting robot Hardware map
        //Robot.init(fileLogger, hardwareMap, robotConfigSettings.robotConfigChoice.TileRunnerRegular, LibraryMotorType.MotorTypes.ANDY20ORBIT);// Starting robot Hardware map

        Robot.allMotorsStop();

        fileLogger.writeEvent(1,"","Wait For Start ");

        dashboard.displayPrintf(1, "Waiting for Start");
        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        dashboard.clearDisplay();

        fileLogger.writeEvent("Starting Loop");

        dashboard.displayPrintf(3, "Controller A Options");
        dashboard.displayPrintf(4, "--------------------");
        dashboard.displayPrintf(8, "Controller B Options");
        dashboard.displayPrintf(9, "--------------------");

        //the main loop.  this is where the action happens
        while (opModeIsActive()) {
            fileLogger.writeEvent(1,"In Main Loop");

                    dashboard.displayPrintf(5, "Controller Mode -- ", "Mecanum Drive Relic Recovery (BAD)");
                    fileLogger.writeEvent(debug,"Controller Mode", "Mecanum Drive Relic Recovery");
                    /*
                     * This was made just for Rover Ruckus becaues the drivers wanted something new!
                     * THis also allows for strafing
                     */
                    Robot.baseMotor1.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor2.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor3.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));
                    Robot.baseMotor4.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -1, 1));

                    dashboard.displayPrintf(1, "baseMotor1 Power", Robot.baseMotor1.getPower());
                    dashboard.displayPrintf(2, "baseMotor2 Power", Robot.baseMotor2.getPower());
                    dashboard.displayPrintf(3, "baseMotor3 Power", Robot.baseMotor3.getPower());
                    dashboard.displayPrintf(4, "baseMotor4 Power", Robot.baseMotor4.getPower());

        }

        //stop the logging
        if (fileLogger != null) {
            fileLogger.writeEvent(1, "TeleOP FINISHED - FINISHED");
            fileLogger.writeEvent(1, "Stopped");
            fileLogger.close();
            fileLogger = null;
        }
    } //RunOpMode
}