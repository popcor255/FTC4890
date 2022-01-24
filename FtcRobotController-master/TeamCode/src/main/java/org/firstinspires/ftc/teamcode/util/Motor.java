class Motor extends DcMotor
{
    public Motor(String name)
    {
        MotorName = name;
    }

    public String MotorName;
    public DcMotor motor;
    public double defaultPower;
    public double defaultDuration;
};