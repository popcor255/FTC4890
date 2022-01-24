class Servo extends CRServo
{
    public Servo(String name)
    {
        super();
        ServoName = name;
    }

    public String ServoName; 
    public CRServo Servo;
    public double defaultPower;
    public double defaultDuration;
};