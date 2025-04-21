import java.io.PrintWriter;
import java.io.FileNotFoundException;
/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * @author ben-moshe
 *
 */
public class Bereshit_101 {
    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FULE = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FULE; // kg
    // https://davidson.weizmann.ac.il/online/askexpert/%D7%90%D7%99%D7%9A-%D7%9E%D7%98%D7%99%D7%A1%D7%99%D7%9D-%D7%97%D7%9C%D7%9C%D7%99%D7%AA-%D7%9C%D7%99%D7%A8%D7%97
    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; //liter per sec, 12 liter per m'
    public static final double SECOND_BURN = 0.009; //liter per sec 0.6 liter per m'
    public static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN;

    public static double accMax(double weight) {
        return acc(weight, true,8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if(main) {t += MAIN_ENG_F;}
        t += seconds*SECOND_ENG_F;
        double ans = t/weight;
        return ans;
    }
    private static void setupTrajectories(TrajectoryInterpolator angleTrajectory, TrajectoryInterpolator vsTrajectory, TrajectoryInterpolator hsTrajectory) {
        // Angle trajectory
        angleTrajectory.addPoint(2000, 58.3); // high alt: tilted
        angleTrajectory.addPoint(600.01, 58.3);
        angleTrajectory.addPoint(600, 45);
        angleTrajectory.addPoint(180.01, 45);
        angleTrajectory.addPoint(180, 30);
        angleTrajectory.addPoint(50.01, 30);
        angleTrajectory.addPoint(20, 0);
        angleTrajectory.addPoint(0, 0);

        // Vertical speed trajectory
        vsTrajectory.addPoint(10000, 30);
        vsTrajectory.addPoint(4000.01, 30);
        vsTrajectory.addPoint(4000.00, 24);
        vsTrajectory.addPoint(2000.01, 24);
        vsTrajectory.addPoint(2000.00, 16);
        vsTrajectory.addPoint(500.01, 16);
        vsTrajectory.addPoint(500.00, 12);
        vsTrajectory.addPoint(100.01, 12);
        vsTrajectory.addPoint(100.00, 6);
        vsTrajectory.addPoint(20.01, 6);
        vsTrajectory.addPoint(20.00, 1);
        vsTrajectory.addPoint(5.01, 1);
        vsTrajectory.addPoint(5.00, 0);

        // Horizontal speed trajectory
        hsTrajectory.addPoint(9989, 765.4);
        hsTrajectory.addPoint(8008, 657.8);
        hsTrajectory.addPoint(5996, 535.9);
        hsTrajectory.addPoint(5006, 470.7);
        hsTrajectory.addPoint(4015, 401.8);
        hsTrajectory.addPoint(3004, 315.6);
        hsTrajectory.addPoint(1994, 214.5);
        hsTrajectory.addPoint(1501, 160.4);
        hsTrajectory.addPoint(1004, 102.2);
        hsTrajectory.addPoint(495, 39.9);
        hsTrajectory.addPoint(249, 12.6);
        hsTrajectory.addPoint(95, 0.0);
        hsTrajectory.addPoint(0, 0.0);
    }


    public static void main(String[] args) {
        PrintWriter writer = null;
        try {
            writer = new PrintWriter("bereshit_log.csv");
        } catch (FileNotFoundException e) {
            System.out.println("Could not create file.");
            return;
        }
        writer.println("time,vs,hs,dist,alt,ang,weight,acc,fuel,dvs,NN");

        System.out.println("Simulating Bereshit's Landing:");
        double vs = 24.8;
        double hs = 932;
        double dist = 181*1000;
        double ang = 58.3; // zero is vertical (as in landing)
        double alt = 13748; // 2:25:40 (as in the simulation) // https://www.youtube.com/watch?v=JJ0VfRL9AMs
        double time = 0;
        double dt = 1; // sec
        double acc=0; // Acceleration rate (m/s^2)
        double fuel = 121; //
        double weight = WEIGHT_EMP + fuel;
        System.out.println("time, vs, hs, dist, alt, ang,weight,acc,fuel,dvs,NN");
        double NN = 0.7; // rate[0,1]

        // Setup PID controllers
        PID vsPID = new PID(0.5, 0.00002, 0.2);     // Vertical speed controller
        PID hsPID = new PID(0.015, 0.00002, 0.5);   // Horizontal speed controller
        PID anglePID = new PID(0.1, 0.0001, 0.25);  // Angle controller

        LandingStates currstate = new LandingStates(vsPID,hsPID);

        // Setup trajectory interpolators
        TrajectoryInterpolator angleTrajectory = new TrajectoryInterpolator();
        TrajectoryInterpolator vsTrajectory = new TrajectoryInterpolator();
        TrajectoryInterpolator hsTrajectory = new TrajectoryInterpolator();

        // Initialize trajectories
        setupTrajectories(angleTrajectory, vsTrajectory, hsTrajectory);
        double hsWeight = 0.0;
        while(alt>0) {
            double desiredVs = vsTrajectory.interpolate(alt);

            if(time%10==0 || alt<100) {
                //System.out.println(time+","+vs+","+hs+","+dist+","+alt+","+ang+","+weight+","+acc+","+fuel);
                String logLine = time + "," + vs + "," + hs + "," + dist + "," + alt + "," + ang + "," + weight + "," + acc + "," + fuel + "," + desiredVs + "," + NN;
                System.out.println(logLine);
                writer.println(logLine);
            }

            currstate.update(alt);
            double nnOutput = vsPID.update(vs-desiredVs, dt);

            double desiredHs = hsTrajectory.interpolate(alt);
            double hsCorrection = hsPID.update(hs - desiredHs, dt);

            // This creates a bell-shaped curve centered at 1200m (middle of 600-1800)
            double distFromMid = Math.abs(alt - 1200) / 600.0;  // Distance from midpoint, normalized
            // Bell curve equation: maximum at center, approaches zero as you move away
            hsWeight = Math.max(0, 1.0 - Math.min(1.0, distFromMid));
            NN = Math.max(0, Math.min(1, (hsCorrection * hsWeight + nnOutput * (1 - hsWeight)) + NN));

            double desiredAngle=angleTrajectory.interpolate(alt);
            double Angle_pid_output = anglePID.update(ang - desiredAngle, dt);
            ang = Math.max(0, Math.min(60, ang-Angle_pid_output));

            double ang_rad = Math.toRadians(ang);
            double h_acc = Math.sin(ang_rad)*acc;
            double v_acc = Math.cos(ang_rad)*acc;
            double vacc = Moon.getAcc(hs);
            time+=dt;
            double dw = dt*ALL_BURN*NN;

            if(fuel>0) {
                fuel -= dw;
                weight = WEIGHT_EMP + fuel;
                acc = NN* accMax(weight);
            }
            else { // ran out of fuel
                acc=0;
            }

            v_acc -= vacc;
            if(hs>2) {hs -= h_acc*dt;}
            dist -= hs*dt;
            vs -= v_acc*dt;
            alt -= dt*vs;
        }
        writer.close();
    }
}
