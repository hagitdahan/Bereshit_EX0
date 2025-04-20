public class LandingStates {
    private PID vsPid;
    private PID hsPid;
    private double hsScale = 0.2;
    public LandingStates(PID vsPid,PID hsPid) {
        this.vsPid = vsPid;
        this.hsPid = hsPid;
    }
    public void update(double alt) {
        if (alt > 2000) {
            vsPid.setGains(0.12, 0.00005, 0.22);
        } else if (alt > 500) {
            vsPid.setGains(0.025, 0.00001, 0.1);
            hsPid.setGains(0.08, 0.0002, 0.15);
        } else {
            vsPid.setGains(0.05, 0.0002, 0.15);
        }
    }
}
