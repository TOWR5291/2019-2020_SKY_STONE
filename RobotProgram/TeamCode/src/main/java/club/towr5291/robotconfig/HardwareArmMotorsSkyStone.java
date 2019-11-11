package club.towr5291.robotconfig;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import club.towr5291.libraries.TOWRDashBoard;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the hardware for a drive base.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */
public class HardwareArmMotorsSkyStone
{
    public ElapsedTime elapse = new ElapsedTime();
    /* Public OpMode members. */
    public DcMotor  liftMotor1        = null;
    public DcMotor  intakeMotor1      = null;
    public DcMotor  tapeMotor         = null;
    public Servo    grabServo         = null;
    public Servo    wristServo        = null;
    public Servo    foundationServo   = null;
    public Servo    rightArmServo     = null;
    public Servo    rightWristServo   = null;
    public Servo    rightClampServo   = null;
    public Servo    leftArmServo      = null;
    public Servo    leftWristServo    = null;
    public Servo    leftClampServo    = null;

    /* local OpMode members. */
    HardwareMap hwMap               =  null;
    private TOWRDashBoard dashBoard = null;

    /* Constructor */
    public HardwareArmMotorsSkyStone(){
        elapse.startTime();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, TOWRDashBoard dash) {

        this.dashBoard = dash;
        this.hwMap = ahwMap;

        // Define and Initialize Motors
        this.liftMotor1         = hwMap.dcMotor.get("liftMotor1");
        this.intakeMotor1       = hwMap.dcMotor.get("intakeMotor1");
        this.tapeMotor          = hwMap.dcMotor.get("tapeMotor");
        this.grabServo          = hwMap.servo.get("grabServo");
        this.wristServo         = hwMap.servo.get("wristServo");
        this.foundationServo    = hwMap.servo.get("foundationServo");
        this.rightArmServo      = hwMap.servo.get("rightArmServo");
        this.rightWristServo    = hwMap.servo.get("rightWristServo");
        this.rightClampServo    = hwMap.servo.get("rightClampServo");
        this.leftArmServo       = hwMap.servo.get("leftArmServo");
        this.leftWristServo     = hwMap.servo.get("leftWristServo");
        this.leftClampServo     = hwMap.servo.get("leftClampServo");

        setHardwareArmDirections();

        liftMotor1.setPower(0);
        this.intakeMotor1.setPower(0);
        this.liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHardwareArmDirections(){
        liftMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        tapeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setHardwareArmDirections(DcMotor.Direction direction){
        liftMotor1.setDirection(direction);
    }

    public void setHardwareLiftMotorResetEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setHardwareLiftMotorRunUsingEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setHardwareLiftMotorRunWithoutEncoders() {
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setHardwareLiftMotorRunToPosition(){
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void allMotorsStop(){
        this.liftMotor1.setPower(0);
        this.intakeMotor1.setPower(0);
        this.tapeMotor.setPower(0);
    }
}