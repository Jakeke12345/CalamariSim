package org.firstinspires.ftc.teamcode.teamcalamari.lib;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;

public class Pos2D {

    private final double x;
    private final double y;
    private final DistanceUnit unit;

    public static final Pos2D ZERO = inches(0,0);

    public static Pos2D inches(double x, double y){
        return new Pos2D(x, y, DistanceUnit.INCH);
    }
    public static Pos2D mms(double x, double y){
        return new Pos2D(x, y, DistanceUnit.MM);
    }

    public Pos2D(double x, double y, DistanceUnit unit){
        this.x = x;
        this.y = y;
        this.unit = unit;
    }

    public double getX(DistanceUnit unit){
        return unit.fromUnit(this.unit, x);
    }
    public double getY(DistanceUnit unit){
        return unit.fromUnit(this.unit, y);
    }

    public Pos2D plus(Pos2D pos){
        return new Pos2D(x+pos.getX(unit), y+pos.getY(unit), unit);
    }
    public Pos2D minus(Pos2D pos){
        return new Pos2D(x-pos.getX(unit), y-pos.getY(unit), unit);
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f) %s", x, y, unit);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Pos2D other)) return false;
        return Double.compare(x, other.x) == 0 &&
                Double.compare(y, other.y) == 0 &&
                unit.equals(other.unit);
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, unit);
    }
}
