
public class Bereshit {

    public static final double WEIGHT_EMP = 165; // kg
    public static final double WEIGHT_FUEL = 420; // kg
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FUEL; // kg

    public static final double MAIN_ENG_F = 430; // N
    public static final double SECOND_ENG_F = 25; // N
    public static final double MAIN_BURN = 0.15; // liter per sec
    public static final double SECOND_BURN = 0.009; // liter per sec
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;

    public static double accMax(double weight) {
        return acc(weight, true, 8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double t = 0;
        if(main) {t += MAIN_ENG_F;}
        t += seconds*SECOND_ENG_F;
        return t/weight;
    }

    // Context Class
    public static class BereshitContext {
        double vs;
        double hs;
        double dist;
        double alt;
        double ang;
        double time;
        double dt = 1;
        double acc;
        double fuel;
        double weight;
        double NN;
        PID pidController;

        LandingState currentState;

        BereshitContext() {
            vs = 24.8;
            hs = 932;
            dist = 181 * 1000;
            alt = 13748;
            ang = 58.3;
            time = 0;
            acc = 0;
            fuel = 121;
            weight = WEIGHT_EMP + fuel;
            NN = 0.7;
            pidController = new PID(0.05, 0.00002, 0.2, 10);
            currentState = new HighAltitudeState();
        }
    }

    // State Interface
    interface LandingState {
        void applyControl(BereshitContext context);
        LandingState getNextState(BereshitContext context);
    }

    // State Implementations
    static class HighAltitudeState implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 30;
            double errorVs = context.vs -  targetVs;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            if (context.alt <= 4000) return new MidAltitudeState();
            return this;
        }
    }

    static class MidAltitudeState implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 24;
            double errorVs = context.vs -  targetVs;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            if (context.alt <= 2000) return new LowAltitudeState();
            return this;
        }
    }

    static class LowAltitudeState implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 16;
            double errorVs = context.vs -  targetVs;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
            //jsjs
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            if (context.alt <= 500) return new FinalDescentState();
            return this;
        }
    }
    static class FinalDescentState implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 12;
            double errorVs =  context.vs- targetVs ;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            if (context.alt <= 100) return new FinalDescentState1();
            return this;
        }
    }
    static class FinalDescentState1 implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 6;
            double errorVs =  context.vs- targetVs ;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            if (context.alt <= 20) return new FinalDescentState2();
            return this;
        }
    }
    static class FinalDescentState2 implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 1 ;
            double errorVs =  context.vs- targetVs ;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            if (context.alt <= 5) return new LandingStateComplete();
            return this;
        }
    }
    static class LandingStateComplete implements LandingState {
        @Override
        public void applyControl(BereshitContext context) {
            double targetVs = 0;
            double errorVs =  context.vs- targetVs ;
            double pidOutput = context.pidController.update(errorVs, context.dt);
            context.NN = Math.max(0, Math.min(1, context.NN+pidOutput));
        }

        @Override
        public LandingState getNextState(BereshitContext context) {
            return this;
        }
    }

    public static void main(String[] args) {
        BereshitContext context = new BereshitContext();
        System.out.println("time, vs, hs, dist, alt, ang, weight, acc, fuel");

        while (context.alt > 0) {
            if (context.time % 10 == 0 || context.alt < 100) {
                System.out.println("time:" + context.time + ", vs: " + context.vs + ", hs: " + context.hs + ", dist:" + context.dist + ", alt: " + context.alt + ", ang: " + context.ang + ", weight: " + context.weight + ", acc:" + context.acc + ", fuel: " + context.fuel);
            }
            context.currentState.applyControl(context);
            context.currentState = context.currentState.getNextState(context);
            if(context.hs<2) {
                context.hs =0; context.ang=0;
            }
            if(context.alt<1){
                context.alt=0;
            }
            double angRad = Math.toRadians(context.ang);
            double hAcc = Math.sin(angRad) * context.acc;
            double vAcc = Math.cos(angRad) * context.acc;
            double vacc = Moon.getAcc(context.hs);

            double dw = context.dt * ALL_BURN * context.NN;
            if (context.fuel > 0) {
                context.fuel -= dw;
                context.weight = WEIGHT_EMP + Math.max(0, context.fuel);
                context.acc = context.NN * accMax(context.weight);
            } else {
                context.acc = 0;
            }
            vAcc -= vacc;
            if (context.hs > 2) {
                context.hs -= hAcc * context.dt;
            }
            context.dist -= context.hs * context.dt;
            context.vs -= vAcc * context.dt;
            context.alt -= context.dt * context.vs;
            context.time += context.dt;
        }
    }
}
