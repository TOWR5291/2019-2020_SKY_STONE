package club.towr5291.functions;

import android.content.SharedPreferences;
import android.os.Environment;
import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import club.towr5291.libraries.LibraryStateSegAuto;
import club.towr5291.libraries.robotConfig;


public class ReadStepFileXML {

    public HashMap<String, LibraryStateSegAuto> autonomousStep = new HashMap<>();
    robotConfig.MOTOR_KIND MOTOR_KIND = null;
    robotConfig.Bases BASE_TYPE = null;

    double dblMotorGearReduction = 0;//How much is the motorGeared down
    double dblDriveGearReduction = 1;//How much is the motor geared down
    double dblWheelDiameterInches = 4;//How big is the wheel
    double dblRobotWidthInches = 18;//How wide is the robot
    double dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = 1;
    double dblCOUNTS_PER_INCH_STRAFE = 1;

    boolean reverseLeftMotor1 = false;
    boolean reverseLeftMotor2 = false;
    boolean reverseRightMotor1 = false;
    boolean reverseRightMotor2 = false;
    boolean usingAdafruitIMU = false;
    boolean usingOpenCV = false;
    boolean usingVuforia = false;
    boolean usingVuforiaWebCam = false;
    int numberOfLoadedSteps = 1;

    private HashMap<String, LibraryStateSegAuto> loadSteps(String fileName){
        File stepFile = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS + "/Sequences"), fileName);
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();

        Log.e("RUNNINNG LOADSTEPS", "STEPS");
        try {
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(stepFile);

            doc.getDocumentElement().normalize();

            System.out.println("Root element :" + doc.getDocumentElement().getNodeName());

            NodeList nStepList = doc.getDocumentElement().getElementsByTagName("Step");
            Element element = doc.getDocumentElement().getAttribute("ID");
//            NodeList nConfigList = doc.getElementsByTagName("Setup");

//            for (int i = 0; i <= nConfigList.getLength(); i++){
//                Node step = nConfigList.item(i);
//                Log.e("Running Pass", step.getPrefix());

//                if (step.getNodeType() == Node.ELEMENT_NODE){
//                    Element eElement = (Element) step;
//                    Log.e("Finished eElement", eElement.getTagName());

                    this.MOTOR_KIND = robotConfig.MOTOR_KIND.toObject(element.getElementsByTagName("MotorKind").item(0).getTextContent());
                    this.BASE_TYPE = robotConfig.Bases.getBaseFromString(element.getElementsByTagName("BaseType").item(0).getTextContent());
                    Log.e("HELP ME", String.valueOf(BASE_TYPE));
                    this.dblMotorGearReduction = Integer.getInteger(element.getElementsByTagName("MotorGearReduction").item(0).getTextContent());
                    this.dblDriveGearReduction = Integer.getInteger(element.getElementsByTagName("DriveGearReduction").item(0).getTextContent());
                    this.dblWheelDiameterInches = Integer.getInteger(element.getElementsByTagName("WheelDiameterInches").item(0).getTextContent());
                    this.dblRobotWidthInches = Integer.getInteger(element.getElementsByTagName("BaseWidth").item(0).getTextContent());
                    this.usingAdafruitIMU = Boolean.parseBoolean(element.getElementsByTagName("EnableIMU").item(0).getTextContent());
                    this.usingOpenCV = Boolean.parseBoolean(element.getElementsByTagName("EnableOpenCV").item(0).getTextContent());
                    this.usingVuforia = Boolean.parseBoolean(element.getElementsByTagName("EnableVuforia").item(0).getTextContent());
                    this.usingVuforiaWebCam = Boolean.parseBoolean(element.getElementsByTagName("EnableVuforiaWebCam").item(0).getTextContent());
                    this.dblCOUNTS_PER_INCH_STRAFE_FRONT_OFFSET = Double.valueOf(element.getElementsByTagName("COUNTS_PER_INCH_STRAFE_FRONT_OFFSET").item(0).getTextContent());
                    this.dblCOUNTS_PER_INCH_STRAFE_REAR_OFFSET = Double.valueOf(element.getElementsByTagName("COUNTS_PER_INCH_STRAFE_REAR_OFFSET").item(0).getTextContent());
                    this.dblCOUNTS_PER_INCH_STRAFE_LEFT_OFFSET = Double.valueOf(element.getElementsByTagName("COUNTS_PER_INCH_STRAFE_LEFT_OFFSET").item(0).getTextContent());
                    this.dblCOUNTS_PER_INCH_STRAFE_RIGHT_OFFSET = Double.valueOf(element.getElementsByTagName("COUNTS_PER_INCH_STRAFE_RIGHT_OFFSET").item(0).getTextContent());
                    this.dblCOUNTS_PER_INCH_STRAFE = Double.valueOf(element.getElementsByTagName("COUNTS_PER_INCH_STRAFE").item(0).getTextContent());

                    if (element.hasAttribute("reverseLeftMotor1")){
                        this.reverseLeftMotor1 = Boolean.valueOf(element.getElementsByTagName("ReverseLeftMotor1").item(0).getTextContent());
                    }
                    if (element.hasAttribute("reverseLeftMotor2")){
                        this.reverseLeftMotor2 = Boolean.valueOf(element.getElementsByTagName("ReverseLeftMotor2").item(0).getTextContent());
                    }
                    if (element.hasAttribute("reverseRightMotor1")){
                        this.reverseRightMotor1 = Boolean.valueOf(element.getElementsByTagName("ReverseRightMotor1").item(0).getTextContent());
                    }
                    if (element.hasAttribute("reverseRightMotor2")){
                        this.reverseRightMotor2 = Boolean.valueOf(element.getElementsByTagName("ReverseRightMotor2").item(0).getTextContent());
                    }
//                }
//            }

            for (int i = 0; i < nStepList.getLength(); i++){
                Node step = nStepList.item(i);

                if (step.getNodeType() == Node.ELEMENT_NODE){
                    Element eElement = (Element) step;

                    this.autonomousStep.put(String.valueOf(numberOfLoadedSteps), new LibraryStateSegAuto(numberOfLoadedSteps,
                            Double.parseDouble(eElement.getElementsByTagName("Timeout").item(0).getTextContent()),
                            eElement.getElementsByTagName("Command").item(0).getTextContent(),
                            Double.parseDouble(eElement.getElementsByTagName("Distance").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Speed").item(0).getTextContent()),
                            Boolean.parseBoolean(eElement.getElementsByTagName("Parallel").item(0).getTextContent()),
                            Boolean.parseBoolean(eElement.getElementsByTagName("Lastpos").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Parm1").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Parm2").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Parm3").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Parm4").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Parm5").item(0).getTextContent()),
                            Double.parseDouble(eElement.getElementsByTagName("Parm6").item(0).getTextContent())));
                }
            }


        } catch (Exception e) {
            e.printStackTrace();
        }

        return this.autonomousStep;
    }

    public HashMap<String, LibraryStateSegAuto> ReadStepFile(SharedPreferences sharedPreferences) {
        HashMap<String, LibraryStateSegAuto> autonomousSteps = new HashMap<String, LibraryStateSegAuto>();
        //load the sequence based on alliance colour and team
        switch (sharedPreferences.getString("club.towr5291.Autonomous.Color", "Red")) {
            case "Red":
                switch (sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left")) {
                    case "Left":
                        switch (sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "5291")) {
                            case "5291":
                                autonomousSteps = loadSteps("5291RedLeftRoverRuckus.xml");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230RedLeftRoverRuckus.xml");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231RedLeftRoverRuckus.xml");
                                break;
                        }
                        break;
                    case "Right":
                        switch (sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "5291")) {
                            case "5291":
                                autonomousSteps = loadSteps("5291RedRightRoverRuckus.xml");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230RedRightRoverRuckus.xml");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231RedRightRoverRuckus.xml");
                                break;
                        }
                        break;
                }
                break;
            case "Blue":
                switch (sharedPreferences.getString("club.towr5291.Autonomous.Position", "Left")) {
                    case "Left":
                        switch (sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "5291")) {
                            case "5291":
                                autonomousSteps = loadSteps("5291BlueLeftRoverRuckus.xml");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230BlueLeftRoverRuckus.xml");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231BlueLeftRoverRuckus.xml");
                                break;
                        }
                        break;
                    case "Right":
                        switch (sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "5291")) {
                            case "5291":
                                autonomousSteps = loadSteps("5291BlueRightRoverRuckus.xml");
                                break;
                            case "11230":
                                autonomousSteps = loadSteps("11230BlueRightRoverRuckus.xml");
                                break;
                            case "11231":
                                autonomousSteps = loadSteps("11231BlueRightRoverRuckus.xml");
                                break;
                        }
                        break;
                }
                break;
            case "Test":
                switch (sharedPreferences.getString("club.towr5291.Autonomous.TeamNumber", "5291")) {
                    case "5291":
                        autonomousSteps = loadSteps("5291TestRoverRuckus.xml");
                        break;
                    case "11230":
                        autonomousSteps = loadSteps("11230TestRoverRuckus.xml");
                        break;
                    case "11231":
                        autonomousSteps = loadSteps("11231TestRoverRuckus.xml");
                        break;
                }

                break;
        }
        autonomousStep = autonomousSteps;
        return autonomousStep;
    }

    public HashMap<String, LibraryStateSegAuto> insertSteps(int timeOut, String command, double distance,  double power, boolean parallel, boolean lastPos, double parm1, double parm2, double parm3, double parm4, double parm5, double parm6, int insertlocation) {
        Log.d("insertSteps", " timout " + timeOut + " command " + command + " distance " + distance + "  power " + power + " parallel " + parallel + " lastPos " + lastPos + " parm1 " + parm1 + " parm2 " + parm2 + " parm3 " + parm3 + " parm4 " + parm4 + " parm5 " + parm5 + " parm6 " + parm6);
        HashMap<String, LibraryStateSegAuto> autonomousStepsTemp = new HashMap<String, LibraryStateSegAuto>();
        LibraryStateSegAuto processingStepsTemp;

        //move all the steps from current step to a temp location
        for (int loop = insertlocation; loop < this.numberOfLoadedSteps; loop++) {
            processingStepsTemp = autonomousStep.get(String.valueOf(loop));
            Log.d("insertSteps", "Reading all the next steps " + loop + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand());
            autonomousStepsTemp.put(String.valueOf(loop), autonomousStep.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "All steps loaded to a temp hasmap");

        //insert the step we want

        autonomousStep.put(String.valueOf(insertlocation), new LibraryStateSegAuto (this.numberOfLoadedSteps, timeOut, command, distance, power, parallel, lastPos, parm1, parm2, parm3, parm4, parm5, parm6));
        Log.d("insertSteps", "Inserted New step");

        //move all the other steps back into the sequence
        for (int loop = insertlocation; loop < this.numberOfLoadedSteps; loop++)
        {
            processingStepsTemp = autonomousStepsTemp.get(String.valueOf(loop));
            Log.d("insertSteps", "adding these steps back steps " + (loop + 1) + " timout " + processingStepsTemp.getmRobotTimeOut() + " command " + processingStepsTemp.getmRobotCommand());
            autonomousStep.put(String.valueOf(loop + 1), autonomousStepsTemp.get(String.valueOf(loop)));
        }
        Log.d("insertSteps", "Re added all the previous steps");
        //increment the step counter as we inserted a new step
        //mValueSteps.add(loadStep, new LibraryStateTrack(false,false));
        this.numberOfLoadedSteps++;
        return autonomousStep;
    }

    public HashMap<String, LibraryStateSegAuto> activeSteps() {
        return autonomousStep;
    }

    public boolean isUsingVuforia() {
        return usingVuforia;
    }
    public void setUsingVuforia(boolean usingVuforia) {
        this.usingVuforia = usingVuforia;
    }

    public boolean isUsingVuforiaWebCam() {
        return usingVuforiaWebCam;
    }
    public void setUsingVuforiaWebCam(boolean usingVuforiaWebCam) {
        this.usingVuforiaWebCam = usingVuforiaWebCam;
    }

    public boolean isUsingOpenCV() {
        return usingOpenCV;
    }
    public void setUsingOpenCV(boolean usingOpenCV) {
        this.usingOpenCV = usingOpenCV;
    }

    public boolean enableAdafruitIMU() {
        return usingAdafruitIMU;
    }
    public void setUsingAdafruitIMU(boolean usingAdafruitIMU) {
        this.usingAdafruitIMU = usingAdafruitIMU;
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

    public HashMap<String, LibraryStateSegAuto> getAutonomousStep() {
        return autonomousStep;
    }
    public void setAutonomousStep(HashMap<String, LibraryStateSegAuto> autonomousStep) {
        this.autonomousStep = autonomousStep;
    }

    public robotConfig.MOTOR_KIND getMotorKind() {
        return MOTOR_KIND;
    }
    public void setMotorKind(robotConfig.MOTOR_KIND MOTOR_KIND) {
        this.MOTOR_KIND = MOTOR_KIND;
    }

    public robotConfig.Bases getBaseKind() {
        return BASE_TYPE;
    }
    public void setBaseKind(robotConfig.Bases BASE_TYPE) {
        this.BASE_TYPE = BASE_TYPE;
    }

    public double getDblMotorGearReduction() {
        return dblMotorGearReduction;
    }
    public void setDblMotorGearReduction(double dblMotorGearReduction) {
        this.dblMotorGearReduction = dblMotorGearReduction;
    }

    public boolean isReverseLeftMotor1() {
        return reverseLeftMotor1;
    }
    public void setReverseLeftMotor1(boolean reverseLeftMotor1) {
        this.reverseLeftMotor1 = reverseLeftMotor1;
    }

    public boolean isReverseLeftMotor2() {
        return reverseLeftMotor2;
    }
    public void setReverseLeftMotor2(boolean reverseLeftMotor2) {
        this.reverseLeftMotor2 = reverseLeftMotor2;
    }

    public boolean isReverseRightMotor1() {
        return reverseRightMotor1;
    }
    public void setReverseRightMotor1(boolean reverseRightMotor1) {
        this.reverseRightMotor1 = reverseRightMotor1;
    }

    public boolean isReverseRightMotor2() {
        return reverseRightMotor2;
    }
    public void setReverseRightMotor2(boolean reverseRightMotor2) {
        this.reverseRightMotor2 = reverseRightMotor2;
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
}