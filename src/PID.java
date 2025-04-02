public class PID {
    private double P, I, D, max_i, integral, last_error;
    private boolean first_run;

    public PID(double p, double i, double d, double max_i) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.max_i = max_i;
        this.integral = 0;
        this.first_run = true;
    }

    public double update(double error, double dt) {
        if (this.first_run) {
            this.last_error = error;
            this.first_run = false;
        }
        this.integral += this.I * error * dt;
        double diff = (error - this.last_error) / dt;
        double const_integral = this.integral;
        double control_out = this.P * error + this.D * diff + const_integral;
        this.last_error = error;
        return control_out;
    }
}
