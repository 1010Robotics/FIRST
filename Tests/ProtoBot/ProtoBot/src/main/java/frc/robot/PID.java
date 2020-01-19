package frc.robot;

public class PID {

    private static final int ERROR = 0;
    private static final int INTEGRAL = 1;
    private static final int DERIVATIVE = 2;
    private static final int PAST_ERROR = 3;
    private static final int INTEGRAL_LIMIT = 4;
    private static final int CONST_INT_VAL = 5;

    private float kP;
    private float kI;
    private float kD;
    public int current;
    public int target;
    public int[] data = new int[7];
    public boolean integralConst;

    public PID() {
        kP = 0;
        kI = 0;
        kD = 0;

        target = 0;
        current = 0;

        data[ERROR] = 0;
        data[INTEGRAL] = 0;
        data[DERIVATIVE] = 0;
        data[PAST_ERROR] = 0;
        data[INTEGRAL_LIMIT] = 0;
        data[CONST_INT_VAL] = 0;
        integralConst = false;
    }

    void set_PID_vars(float kP, float kI, float kD, int integral_limit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        data[INTEGRAL_LIMIT] = integral_limit;
    }

    void set_PID_vars(float kP, int const_kI, float kD, int integral_limit, boolean const_int) {
        this.kP = kP;
        this.kD = kD;
        integralConst = const_int;
        data[INTEGRAL_LIMIT] = integral_limit;
        data[CONST_INT_VAL] = const_kI;
    }


    double output(double max_power) {
        double out;

        data[PAST_ERROR] = data[ERROR];

        data[ERROR] = target - current;

        data[DERIVATIVE] = data[ERROR] - data[PAST_ERROR];

        if(integralConst == true) { 
            data[INTEGRAL] = Math.abs(data[INTEGRAL]) < data[INTEGRAL_LIMIT] ? (data[INTEGRAL] = data[INTEGRAL] + data[ERROR]) : (data[INTEGRAL] = 0);
        }
        else {
            data[INTEGRAL] = data[CONST_INT_VAL];
        }

        out = (double)(data[ERROR] * kP) + (double)(data[INTEGRAL] * kI) + (double)(data[DERIVATIVE] * kD);

        out = out > max_power ? max_power : -out < -max_power ? -max_power : out;

        return out;
    }


}