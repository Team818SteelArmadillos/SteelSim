using UnityEngine;
using UnityEngine.Rendering;

public class PIDController : MonoBehaviour
{
    float kP = 0;
    float kI = 0;
    float kD = 0;

    float targetPoint = 0;
    float currentPoint = 0;
    float lastPoint = 0;
    float totalError = 0;
    float derivativeError = 0;
    public delegate float CurrentPoint();
    CurrentPoint currentPointDelegate; 

    public PIDController(float kP, float kI, float kD, CurrentPoint currentPointDelegate)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.currentPointDelegate = currentPointDelegate;
    }

    public void setTarget(float targetPoint)
    {
        this.targetPoint = targetPoint;
        totalError = 0;
    }

    public void setkP(float kP)
    {
        this.kP = kP;
    }

    public void setkI(float kI)
    {
        this.kI = kI;
    }

    public void setkD(float kD)
    {
        this.kD = kD;
    }
    public float calculate()
    {
        return (targetPoint-currentPoint)*kP + totalError*kI + derivativeError*kD;
    }

    private void FixedUpdate()
    {

        currentPoint = currentPointDelegate();
        totalError += currentPoint * Time.fixedDeltaTime;
        derivativeError = (currentPoint - lastPoint) / Time.fixedDeltaTime;
        lastPoint = currentPoint;
    }
}
