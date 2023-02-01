package frc.robot;

public class vector {

    private double vectorX;
    private double vectorY;
    private double vectorAng;
    private double vectorMag;

    /**
     * @param cordanateType vector or cartesian
     * @param A for vector- ang. for cartesian- X
     * @param B for vector- mag. for cartesian- Y
     */
    public vector(String cordanateType, double A, double B) {
      switch (cordanateType) {
          case "vector":  
              vectorX = Math.cos(A) * B;
              vectorY = Math.sin(A) * B;
              vectorAng = A;
              vectorMag = B;
          case "cartesian":  
              vectorX = A;
              vectorY = B;
              vectorAng = Math.atan2(B, A);
              vectorMag = Math.hypot(A, B);
      break;
      default: break;
      }
    } 
    
    public double getX(){
        return vectorX;
    }

    public double getY(){
        return vectorY;
    }

    public double getAng(){
        return vectorAng;
    }

    public double getMag(){
        return vectorMag;
    }
}
