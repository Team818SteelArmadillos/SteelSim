using UnityEngine;

public class MotorController : MonoBehaviour
{
    public enum ControlMode
    {
        Duty,
        Velocity,
        Position
    }

    [Header("References")]
    public DCMotor motor;
    public Encoder encoder;

    [Header("Control Mode")]
    public ControlMode mode = ControlMode.Duty;

    [Header("Setpoints")]
    public float dutySetpoint;
    public float velocitySetpoint;   // rad/s
    public float positionSetpoint;   // radians

    [Header("PID Controllers")]
    public PIDController velocityPID = new PIDController();
    public PIDController positionPID = new PIDController();

    [Header("Feedforward")]
    public float kS = 0f;  // static friction feedforward
    public float kV = 0f;  // velocity feedforward
    public float kA = 0f;  // acceleration feedforward

    [Header("Limits")]
    public float maxDuty = 1f;
    public float rampRate = 5f;  // duty units per second
    public float currentLimit = 100f;

    [Header("Brake Mode")]
    public bool brakeMode = true;

    private float previousVelocity;
    private float commandedDuty;

    void Awake()
    {
        encoder = motor.getEncoder();
        // Ensure PID objects exist
        if (velocityPID == null)
            velocityPID = new PIDController();

        if (positionPID == null)
            positionPID = new PIDController();

        // Reset internal state
        velocityPID.Reset();
        positionPID.Reset();
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        float measuredPosition = encoder.GetAngleRadians();
        float measuredVelocity = encoder.GetVelocity();
        Debug.Log(measuredPosition);
        float targetDuty = 0f;

        switch (mode)
        {
            case ControlMode.Duty:
                targetDuty = dutySetpoint;
                break;

            case ControlMode.Velocity:
                float velocityOutput = velocityPID.Update(
                    velocitySetpoint,
                    measuredVelocity,
                    dt);

                float velocityFF =
                    kS * Mathf.Sign(velocitySetpoint) +
                    kV * velocitySetpoint;

                targetDuty = velocityOutput + velocityFF;
                break;

            case ControlMode.Position:
                float velocityTarget =
                    positionPID.Update(positionSetpoint, measuredPosition, dt);

                float posVelocityOutput =
                    velocityPID.Update(velocityTarget, measuredVelocity, dt);

                targetDuty = posVelocityOutput;
                break;
        }

        // Ramp limiting
        float maxStep = rampRate * dt;
        targetDuty = Mathf.Clamp(
            targetDuty,
            commandedDuty - maxStep,
            commandedDuty + maxStep);

        // Clamp output
        commandedDuty = Mathf.Clamp(targetDuty, -maxDuty, maxDuty);

        // Current limiting
        if (Mathf.Abs(motor.GetCurrent()) > currentLimit)
        {
            commandedDuty *= 0.9f;
        }

        // Brake vs Coast
        if (!brakeMode && Mathf.Approximately(commandedDuty, 0f))
        {
            motor.dutyCycle = 0f;
        }
        else
        {
            motor.dutyCycle = commandedDuty;
        }

        previousVelocity = measuredVelocity;
    }
}