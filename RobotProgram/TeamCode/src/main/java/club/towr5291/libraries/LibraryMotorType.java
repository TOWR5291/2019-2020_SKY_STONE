package club.towr5291.libraries;

public class LibraryMotorType {

    public double GEARRATIO;
    public double COUNTSPERROTATION;
    public double PULSEPERROTATION;

    private double dblENCODER_CPR_REV40SPUR = 1120;
    private double dblENCODER_CPR_REV20SPUR = 560;
    private double dblENCODER_CPR_REV20ORBITAL = 560;
    private double dblENCODER_CPR_ANDY20ORBITAL = 537.6;
    private double dblENCODER_CPR_ANDY20SPUR = 560;
    private double dblENCODER_CPR_ANDY40SPUR = 1120;
    private double dblENCODER_CPR_ANDY60SPUR = 1680;
    private double dblENCODER_CPR_ANDY3_7ORBITAL = 0;

    private double dblENCODER_PPR_REV40SPUR = 112;
    private double dblENCODER_PPR_REV20SPUR = 56;
    private double dblENCODER_PPR_REV20ORBITAL = 56;
    private double dblENCODER_PPR_ANDY20ORBITAL = 134.4;
    private double dblENCODER_PPR_ANDY20SPUR = 28;
    private double dblENCODER_PPR_ANDY40SPUR = 28;
    private double dblENCODER_PPR_ANDY60SPUR = 28;
    private double dblENCODER_PPR_ANDY3_7ORBITAL = 0;

    private double dblGearRatioREV40SPUR = 40;
    private double dblGearRatioREV20SPUR = 20;
    private double dblGearRatioREV20ORBITAL = 20;
    private double dblGearRatioANDY20ORBITAL = 19.2;
    private double dblGearRatioANDY20SPUR = 20;
    private double dblGearRatioANDY40SPUR = 40;
    private double dblGearRatioANDY60SPUR = 60;
    private double dblGearRatioANDY3_7ORBITAL = 3.7;

    public enum MotorTypes {
        REV ("REV"),
        ANDY ("ANDY"),
        REV_CHANGEABLE ("REV_CHANGEABLE");

        public String name;

        MotorTypes(String name){
            this.name = name;
        }

        public String toString() {
            return this.name;
        }
    }

    public enum REV_VARIABLE_RATIOS {
        R1_5 (5, "1:5"),
        R1_4 (4, "1:4"),
        R1_3 (3, "1:3");

        public double value;
        public String name;

        REV_VARIABLE_RATIOS(double value, String name){this.value = value; this.name = name;}

        public double getValue(){return this.value;}
        public String getName(){return this.name;}
    }

    public enum REV_RATIO {
        R1_20,
        R
    }

    public LibraryMotorType(){
        //Nothing In here yet :)
    }

}