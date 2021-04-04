package org.firstinspires.ftc.teamcode.test;

public class FieldCoordinate {
    volatile public double x = 0.0;
    volatile public double y = 0.0;

    public FieldCoordinate(){}

    public FieldCoordinate(double x, double y){
        this.x = x;
        this.y = y;
    }

    public static FieldCoordinate createInstance(){
        return new FieldCoordinate();
    }

    public double distance(FieldCoordinate target) {
        return Math.sqrt(Math.pow(target.x-x,2)+Math.pow(target.y-y,2));
    }


}
