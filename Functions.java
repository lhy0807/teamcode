package org.firstinspires.ftc.teamcode;

public class Functions {
    /**modify stick input
     * @param Var raw stick input
     * @param deadzone true to implement deadzone
     * @param square_Input true to use square input
     */
    protected static double stickMod(double Var, boolean deadzone, boolean square_Input){
        double tVar=Var;
        double tDeadzone=(deadzone)? RoboMap.dDeadzone:0;
        double temp;
        if(tVar<=tDeadzone&&tVar>=-tDeadzone){
            return 0;
        }else{
            if(tVar>0){
                return (!square_Input)?((tVar-tDeadzone)/(1-tDeadzone)):
                        ((tVar-tDeadzone)/(1-tDeadzone)*(tVar-tDeadzone)/(1-tDeadzone));
            }else {
                return (!square_Input)?((tVar+tDeadzone)/(1-tDeadzone)):
                        ((tVar+tDeadzone)/(1-tDeadzone)*(tVar+tDeadzone)/(1-tDeadzone));
            }
        }
    }

    /**Mecanum Drive, returns motor speed [-1,1]*/
    protected static double MecDrive_RightFront(double x, double y, double z, double w){
        return (-RoboMap.pX*x+RoboMap.pY*y-RoboMap.pZ*z+RoboMap.pW*w);
    }

    /**Mecanum Drive, returns motor speed [-1,1]*/
    protected static double MecDrive_LeftFront(double x, double y, double z, double w){
        return (RoboMap.pX*x+RoboMap.pY*y+RoboMap.pZ*z+RoboMap.pW*w);
    }

    /**Mecanum Drive, returns motor speed [-1,1]*/
    protected static double MecDrive_LeftRear(double x, double y, double z, double w){
        return (-RoboMap.pX*x+RoboMap.pY*y+RoboMap.pZ*z+RoboMap.pW*w);
    }

    /**Mecanum Drive, returns motor speed [-1,1]*/
    protected static double MecDrive_RightRear(double x, double y, double z, double w){
        return (RoboMap.pX*x+RoboMap.pY*y-RoboMap.pZ*z+RoboMap.pW*w);
    }

    /**Stick output
     * x - Left X
     * y - Left Y
     * z - Right X
     * w - Right Y
     */
}
