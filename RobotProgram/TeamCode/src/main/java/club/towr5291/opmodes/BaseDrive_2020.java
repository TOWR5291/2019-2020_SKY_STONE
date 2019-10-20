package club.towr5291.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class BaseDrive_2020 extends LinearOpMode {


    public DcMotor intakeMotor1 = null;
    public DcMotor liftMotor1 = null;
    public Servo clawServo = null;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeMotor1 = hardwareMap.get(DcMotor.class, "intakeMotor1");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        waitForStart();

        while(opModeIsActive()){
            intakeMotor1.setPower(gamepad2.left_trigger);
            liftMotor1.setPower(-gamepad2.left_stick_y);

            if (gamepad2.a) clawServo.setPosition(clawServo.getPosition() + .1);
            if (gamepad2.b) clawServo.setPosition(clawServo.getPosition() - .1);
        }
    }
}
