using UnityEngine;

[System.Serializable]
public class Encoder : MonoBehaviour
{
    [Header("Resolution")]
    public int countsPerRevolution = 2048;

    [Header("Mounting")]
    public DCMotor motor;
    public bool mountedOnMotorShaft = true;
    public float gearRatio = 10f;   // Same as motor gear ratio

    [Header("Noise")]
    public float positionNoiseStdDev = 0f;  // radians
    public float velocityNoiseStdDev = 0f;  // rad/s

    private int tickCount;
    private float previousAngle;
    private float velocity;
    private bool firstUpdate = true;

    public float motorAngle = 0f;

    // Optional latency simulation
    public float measurementDelay = 0f;
    private float delayedAngle;

    public void Reset()
    {
        tickCount = 0;
        previousAngle = 0f;
        velocity = 0f;
        firstUpdate = true;
    }

    public void FixedUpdate()
    {

        float dt = Time.fixedDeltaTime;
        motorAngle += motor.GetMotorSpeed() * dt;
        float outputAngle = motorAngle/motor.gearRatio;
        float trueAngle = mountedOnMotorShaft ? motorAngle : outputAngle;

        // Add optional measurement delay (simple lag)
        delayedAngle = Mathf.Lerp(delayedAngle, trueAngle, dt / Mathf.Max(measurementDelay, 1e-6f));

        float measuredAngle = delayedAngle;

        // Add position noise
        if (positionNoiseStdDev > 0f)
            measuredAngle += RandomGaussian() * positionNoiseStdDev;

        // Compute tick count (quantized)
        float revolutions = measuredAngle / (2f * Mathf.PI);
        tickCount = Mathf.RoundToInt(revolutions * countsPerRevolution);

        // Velocity estimate (finite difference)
        if (!firstUpdate)
        {
            velocity = (measuredAngle - previousAngle) / dt;

            if (velocityNoiseStdDev > 0f)
                velocity += RandomGaussian() * velocityNoiseStdDev;
        }

        previousAngle = measuredAngle;
        firstUpdate = false;
    }

    public int GetTicks() => tickCount;

    public float GetAngleRadians()
    {
        return (float)tickCount / countsPerRevolution * 2f * Mathf.PI;
    }

    public float GetRevolutions()
    {
        return (float)tickCount / countsPerRevolution;
    }

    public float GetVelocity() => velocity;

    // Gaussian noise generator (Box-Muller)
    private float RandomGaussian()
    {
        float u1 = Random.value;
        float u2 = Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u1)) * Mathf.Cos(2f * Mathf.PI * u2);
    }
}