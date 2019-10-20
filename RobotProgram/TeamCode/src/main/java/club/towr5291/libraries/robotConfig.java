package club.towr5291.libraries;

import android.content.SharedPreferences;
import android.util.Log;

import club.towr5291.functions.ReadStepFileXML;

import static club.towr5291.functions.Constants.SharedPreferencesValues.ALLIANCE_COLOR;
import static club.towr5291.functions.Constants.SharedPreferencesValues.ALLIANCE_START_POSITION;
import static club.towr5291.functions.Constants.SharedPreferencesValues.DEBUG;
import static club.towr5291.functions.Constants.SharedPreferencesValues.START_DELAY;
import static club.towr5291.functions.Constants.SharedPreferencesValues.TEAM_NUMBER;
import static club.towr5291.libraries.robotConfig.MOTOR_KIND.REV_INTERCHANGEABLE;

/**
 * Created by Ian Haden on 02/06/2018.
 *
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Modification history
 * Edited by:
 * Ian Haden    02/06/2018  -> Initial creation
 * Wyatt Ashley 03/05/2019  -> Changed A LOT configuration is now different have another class for motor types
 * Wyatt Ashley 04/06/2019  -> Cleaned all of the code and is now easier to use
 */

public class robotConfig {

    private String teamNumber;
    private String allianceColor;
    private String allianceStartPosition;
    private int delay;
    private int debug;
//    private String robotConfigBase;
//    private String robotMotorType;
//    private double robotMotorRatio;

    private boolean reverseLeftMotor1;
    private boolean reverseRightMotor1;
    private boolean reverseLeftMotor2;
    private boolean reverseRightMotor2;
    private boolean enableAdafriutIMU = false;
    private boolean enableOpenCV = false;
    private boolean enableVuforia = false;
    private boolean enableVuforiaWebCam = false;

    private robotConfig.MOTOR_KIND MOTOR_KIND = REV_INTERCHANGEABLE;
    robotConfig.Bases BASE_TYPE = Bases.MECANUM_2020;

    private double dblMotorGearReduction = 15;
    private double dblDriveGearReduction = 1;
    private double dblWheelDiameterInches = 4;
    private double dblRobotWidthInches = 18;
    private double dblCountsPerInches = 0;
    private double dblCountsPerDegree = 0;

    double dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE = 1;
//    //set up robot variables
//    private double COUNTS_PER_MOTOR_REV;                                        // eg: TETRIX = 1440 pulses, NeveRest 20 = 560 pulses, NeveRest 40 =  1120, NeveRest 60 = 1680 pulses
//    private double WHEEL_DIAMETER_INCHES;                                       // For figuring circumference
//    private double WHEEL_ACTUAL_FUDGE;                                          // Fine tuning amount
//    private double COUNTS_PER_INCH;
//    private double ROBOT_TRACK;                                                 //  distance between centerline of rear wheels robot will pivot on rear wheel of omni on front, 16.5 track is 103.67 inches full circle
//    private double COUNTS_PER_DEGREE;
//    private double WHEEL_TURN_FUDGE;
//    private double REVERSE_DIRECTION;                                           // determines which direction the robot runs when FW is positive or negative when commanded to move a direction
//    private int    LIFTMAIN_COUNTS_PER_INCH;                                    // number of encoder counts oer inch
//    private double COUNTS_PER_INCH_STRAFE;
//    private double COUNTS_PER_INCH_STRAFE_FRONT_OFFSET;
//    private double COUNTS_PER_INCH_STRAFE_REAR_OFFSET;
//    private double COUNTS_PER_INCH_STRAFE_LEFT_OFFSET;
//    private double COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET;
//    private double MECANUM_TURN_OFFSET;

    public enum Bases {
        MECANUM_2019("MECANUM_2019"),
        MECANUM_2020("MECANUM_2020"),
        MECANUM_TILERUNNER("MECANUM_TILERUNNER"),
        OMNI_TILERUNNER("OMNI_TILERUNNER"),
        TILERUNNER("TILERUNNER");

        public String value = "default";

        Bases (String name){
            this.value = name;
        }

        public String toString(){
            return this.value;
        }

        public static Bases getBaseFromString(String name){
            switch(name.toUpperCase()){
                case "MECANUM_2019":
                    return MECANUM_2019;
                case "MECANUM_2020":
                    return  MECANUM_2020;
                case "MECANUM_TILERUNNER":
                    return MECANUM_TILERUNNER;
                case "OMNI_TILERUNNER":
                    return OMNI_TILERUNNER;
                case "TILERUNNER":
                    return TILERUNNER;
                default:
                    return MECANUM_2019;
            }
        }
    }

    public enum MOTOR_KIND {
        ANDY ("ANDY"),
        REV_REGULAR ("REV_REGULAR"),
        REV_INTERCHANGEABLE ("REV_INTERCHANGEABLE");

        public String value = "default";

        MOTOR_KIND(String name){
            this.value = name;
        }

        public String toString(){
            return this.value;
        }

        public static MOTOR_KIND toObject(String name){
            switch (name.toUpperCase()){
                case "ANDY":
                    return ANDY;
                case "REV_INTERCHANGEABLE":
                    return REV_INTERCHANGEABLE;
                case "REV_REGULAR":
                    return REV_REGULAR;
                default:
                    return REV_REGULAR;
            }
        }
    }

    public enum motors {
        leftMotor1 ("leftMotor1", 1),
        leftMotor2 ("leftMotor2", 2),
        rightMotor1 ("rightMotor1", 4),
        rightMotor2 ("rightMotor2", 8);

        private final String name;
        private final int value;

        motors (String name, int value) {
            this.name = name;
            this.value = value;
        }


        public String toString() {
            return name;
        }

        public int toInt() {
            return value;
        }
    }

    public enum LEDnames {
        leftGreen ("green1", 1),
        leftRed ("red1", 2),
        leftBlue ("blue1", 4),
        rightGreen ("green2", 8),
        rightRed ("red2", 16),
        rightBlue ("blue2", 32);

        private final String name;
        private final int value;

        LEDnames (String name, int value) {
            this.name = name;
            this.value = value;
        }

        public String toString() {
            return name;
        }

        public int toInt() {
            return value;
        }
    }


    public robotConfig (SharedPreferences sharedPreferences, ReadStepFileXML readStepFileXML){
        this.allianceColor = sharedPreferences.getString(ALLIANCE_COLOR.getSharedPrefString(), ALLIANCE_COLOR.getSharedPrefDefault());// Using a Function to Store The Robot Specification
        this.teamNumber = sharedPreferences.getString(TEAM_NUMBER.getSharedPrefString(), TEAM_NUMBER.getSharedPrefDefault());
        this.allianceStartPosition = sharedPreferences.getString(ALLIANCE_START_POSITION.getSharedPrefString(), ALLIANCE_START_POSITION.getSharedPrefDefault());
        this.delay = Integer.parseInt(sharedPreferences.getString(START_DELAY.getSharedPrefString(), START_DELAY.getSharedPrefDefault()));
        this.debug = Integer.parseInt(sharedPreferences.getString(DEBUG.getSharedPrefString(), DEBUG.getSharedPrefDefault()));

        this.dblWheelDiameterInches = readStepFileXML.getDblWheelDiameterInches();
        this.dblMotorGearReduction = readStepFileXML.getDblMotorGearReduction();
        this.dblDriveGearReduction = readStepFileXML.getDblDriveGearReduction();
        this.dblRobotWidthInches = readStepFileXML.getDblRobotWidthInches();
        this.BASE_TYPE = readStepFileXML.getBaseKind();
        //Log.e("HELP ME", String.valueOf(readStepFileXML.getBaseKind()));
        this.MOTOR_KIND = readStepFileXML.getMotorKind();
        this.enableAdafriutIMU = readStepFileXML.enableAdafruitIMU();
        this.enableOpenCV = readStepFileXML.isUsingOpenCV();
        this.enableVuforia = readStepFileXML.isUsingVuforia();
        this.enableVuforiaWebCam = readStepFileXML.isUsingVuforiaWebCam();

        this.dblCountsPerInches = ((this.dblMotorGearReduction * this.dblDriveGearReduction) / (this.dblWheelDiameterInches * 3.1415));
        this.dblCountsPerDegree = ((2 * 3.1415 * this.dblRobotWidthInches) * this.dblCountsPerInches) / 360;;


        this.dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET = readStepFileXML.getDblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET();
        this.dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET = readStepFileXML.getDblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET();
        this.dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET = readStepFileXML.getDblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET();
        this.dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = readStepFileXML.getDblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET();
        this.dblCOUNTS_PER_INCH_STRAFE = readStepFileXML.getDblCOUNTS_PER_INCH_STRAFE();
    }

    public double getDblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET() {
        return dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET;
    }

    public void setDblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET(double dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET) {
        this.dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET = dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET;
    }

    public double getDblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET() {
        return dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET;
    }

    public void setDblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET(double dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET) {
        this.dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET = dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET;
    }

    public double getDblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET() {
        return dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET;
    }

    public void setDblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET(double dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET) {
        this.dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET = dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET;
    }

    public double getDblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET() {
        return dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET;
    }

    public void setDblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET(double dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET) {
        this.dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET;
    }

    public double getDblCOUNTS_PER_INCH_STRAFE() {
        return dblCOUNTS_PER_INCH_STRAFE;
    }

    public void setDblCOUNTS_PER_INCH_STRAFE(double dblCOUNTS_PER_INCH_STRAFE) {
        this.dblCOUNTS_PER_INCH_STRAFE = dblCOUNTS_PER_INCH_STRAFE;
    }

    public double getDblCountsPerDegree() {
        return dblCountsPerDegree;
    }
    public void setDblCountsPerDegree(double dblCountsPerDegree) {
        this.dblCountsPerDegree = dblCountsPerDegree;
    }

    public boolean isEnableVuforia() {
        return enableVuforia;
    }
    public void setEnableVuforia(boolean enableVuforia) {
        this.enableVuforia = enableVuforia;
    }

    public boolean isEnableVuforiaWebCam() {
        return enableVuforiaWebCam;
    }
    public void setEnableVuforiaWebCam(boolean enableVuforiaWebCam) {
        this.enableVuforiaWebCam = enableVuforiaWebCam;
    }

    public boolean isEnableOpenCV() {
        return enableOpenCV;
    }
    public void setEnableOpenCV(boolean enableOpenCV) {
        this.enableOpenCV = enableOpenCV;
    }

    public boolean isEnableAdafriutIMU() {
        return enableAdafriutIMU;
    }
    public void setEnableAdafriutIMU(boolean enableAdafriutIMU) {
        this.enableAdafriutIMU = enableAdafriutIMU;
    }

    public String getTeamNumber() {
        return teamNumber;
    }
    public void setTeamNumber(String teamNumber) {
        this.teamNumber = teamNumber;
    }

    public String getAllianceColor() {
        return allianceColor;
    }
    public void setAllianceColor(String allianceColor) {
        this.allianceColor = allianceColor;
    }

    public String getAllianceStartPosition() {
        return allianceStartPosition;
    }
    public void setAllianceStartPosition(String allianceStartPosition) {
        this.allianceStartPosition = allianceStartPosition;
    }

    public int getDelay() {
        return delay;
    }
    public void setDelay(int delay) {
        this.delay = delay;
    }

    public int getDebug() {
        return debug;
    }
    public void setDebug(int debug) {
        this.debug = debug;
    }

    public boolean isReverseLeftMotor1() {
        return reverseLeftMotor1;
    }
    public void setReverseLeftMotor1(boolean reverseLeftMotor1) {
        this.reverseLeftMotor1 = reverseLeftMotor1;
    }

    public boolean isReverseRightMotor1() {
        return reverseRightMotor1;
    }
    public void setReverseRightMotor1(boolean reverseRightMotor1) {
        this.reverseRightMotor1 = reverseRightMotor1;
    }

    public boolean isReverseLeftMotor2() {
        return reverseLeftMotor2;
    }
    public void setReverseLeftMotor2(boolean reverseLeftMotor2) {
        this.reverseLeftMotor2 = reverseLeftMotor2;
    }

    public boolean isReverseRightMotor2() {
        return reverseRightMotor2;
    }
    public void setReverseRightMotor2(boolean reverseRightMotor2) {
        this.reverseRightMotor2 = reverseRightMotor2;
    }

    public robotConfig.MOTOR_KIND getMOTOR_KIND() {
        return MOTOR_KIND;
    }
    public void setMOTOR_KIND(robotConfig.MOTOR_KIND MOTOR_KIND) {
        this.MOTOR_KIND = MOTOR_KIND;
    }

    public Bases getBASE_TYPE() {
        return BASE_TYPE;
    }
    public void setBASE_TYPE(Bases BASE_TYPE) {
        this.BASE_TYPE = BASE_TYPE;
    }

    public double getDblMotorGearReduction() {
        return dblMotorGearReduction;
    }
    public void setDblMotorGearReduction(double dblMotorGearReduction) {
        this.dblMotorGearReduction = dblMotorGearReduction;
    }

    public double getDblDriveGearReduction() {
        return dblDriveGearReduction;
    }
    public void setDblDriveGearReduction(double dblDriveGearReduction) {
        this.dblDriveGearReduction = dblDriveGearReduction;
    }

    public double getDblWheelDiameterInches() {
        return dblWheelDiameterInches;
    }
    public void setDblWheelDiameterInches(double dblWheelDiameterInches) {
        this.dblWheelDiameterInches = dblWheelDiameterInches;
    }

    public double getDblRobotWidthInches() {
        return dblRobotWidthInches;
    }
    public void setDblRobotWidthInches(double dblRobotWidthInches) {
        this.dblRobotWidthInches = dblRobotWidthInches;
    }

    public double getCOUNTS_PER_INCH(){
        return this.dblCountsPerInches;
    }
    public void setCOUNTS_PER_INCH(double value){
        this.dblCountsPerInches = value;
    }
}