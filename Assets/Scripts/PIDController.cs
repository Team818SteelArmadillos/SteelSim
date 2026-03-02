using UnityEngine;

[System.Serializable]
public class PIDController
{
    public float Kp;
    public float Ki;
    public float Kd;

    public float outputMin = -1f;
    public float outputMax = 1f;
    public float integralMin = -1f;
    public float integralMax = 1f;

    private float integral;
    private float previousMeasurement;
    private bool initialized;


    public void Reset()
    {
        integral = 0f;
        previousMeasurement = 0f;
        initialized = false;
    }

    public float Update(float setpoint, float measurement, float dt)
    {
        float error = setpoint - measurement;

        float P = Kp * error;

        // Integral
        integral += Ki * error * dt;
        integral = Mathf.Clamp(integral, integralMin, integralMax);

        // Derivative on measurement
        float derivative = 0f;
        if (initialized)
        {
            derivative = (measurement - previousMeasurement) / dt;
        }

        float D = -Kd * derivative;

        previousMeasurement = measurement;
        initialized = true;

        float output = P + integral + D;
        return Mathf.Clamp(output, outputMin, outputMax);
    }
}