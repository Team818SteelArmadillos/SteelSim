using UnityEngine;

/// <summary>
/// DC Motor Physics Simulation
/// Models a brushed DC motor using real electrical and mechanical equations.
///
/// Key equations:
///   V = IR + L*(dI/dt) + Ke*ω       (Kirchhoff voltage law)
///   τ = Kt*I                          (motor torque)
///   J*(dω/dt) = τ - B*ω - τ_load    (Newton's 2nd law for rotation)
///
/// Integration method: Exact Zero-Order-Hold (ZOH) exponential solution.
///   For a first-order linear ODE  dx/dt = a·x + b  the exact solution is:
///     x(t+dt) = x(t)·e^(a·dt) + (b/a)·(e^(a·dt) - 1)
///   This is unconditionally stable for any timestep, making it safe to use
///   even when dt >> τ (the system time constant).  Euler integration becomes
///   unstable when dt > 2τ, which happens routinely with the tiny electrical
///   time constant τ_L = L/R (often < 1 ms) at Unity's default 50 Hz FixedUpdate.
///
///   The two subsystems solved exactly each step:
///     Electrical:  dI/dt  = -(R/L)·I  + (V - Ke·ω)/L   →  a = -R/L,  b = (V-Ke·ω)/L
///     Mechanical:  dω/dt  = -(B/J)·ω  + (Kt·I - τ_load)/J  →  a = -B/J, b = net/J
///
/// Units: SI throughout (V, A, Ω, H, N·m, rad/s, kg·m²)
/// </summary>
public class DCMotor : MonoBehaviour
{
    [Header("Electrical Parameters")]
    [Tooltip("Armature resistance (Ohms)")]
    public float resistance = 1.0f;         // R — typical: 0.5–5 Ω

    [Tooltip("Armature inductance (Henries)")]
    public float inductance = 0.001f;       // L — typical: 0.5–5 mH

    [Tooltip("Back-EMF constant (V·s/rad)")]
    public float backEmfConstant = 0.05f;   // Ke

    [Header("Mechanical Parameters")]
    [Tooltip("Motor torque constant (N·m/A)")]
    public float torqueConstant = 0.05f;    // Kt (ideally == Ke)

    [Tooltip("Rotor moment of inertia (kg·m²)")]
    public float inertia = 0.0001f;         // J

    [Tooltip("Viscous friction coefficient (N·m·s/rad)")]
    public float viscousFriction = 0.0005f; // B

    [Tooltip("Static / Coulomb friction torque (N·m)")]
    public float staticFriction = 0.002f;   // τ_c

    [Header("Thermal (optional)")]
    [Tooltip("Thermal resistance (°C/W)")]
    public float thermalResistance = 5f;

    [Tooltip("Thermal capacitance (J/°C)")]
    public float thermalCapacitance = 20f;

    [Tooltip("Ambient temperature (°C)")]
    public float ambientTemperature = 25f;

    [Header("Runtime State — Read Only")]
    [SerializeField] private float _voltage;        // Applied voltage (V)
    [SerializeField] private float _current;        // Armature current (A)
    [SerializeField] private float _angularVelocity;// ω  (rad/s)
    [SerializeField] private float _angle;          // θ  (rad, cumulative)
    [SerializeField] private float _torque;         // Output torque (N·m)
    [SerializeField] private float _temperature;    // Winding temperature (°C)
    [SerializeField] private float _powerInput;     // Electrical power in (W)
    [SerializeField] private float _powerOutput;    // Mechanical power out (W)
    [SerializeField] private float _efficiency;     // η (0–1)

    // External load torque set by motor controller or other components
    private float _loadTorque;

    // ── Public accessors ───────────────────────────────────────────────
    public float Voltage           => _voltage;
    public float Current           => _current;
    public float AngularVelocity   => _angularVelocity;   // rad/s
    public float RPM               => _angularVelocity * 9.5492f; // rad/s → RPM
    public float AngleRad          => _angle;
    public float AngleDeg          => _angle * Mathf.Rad2Deg;
    public float Torque            => _torque;
    public float Temperature       => _temperature;
    public float PowerInput        => _powerInput;
    public float PowerOutput       => _powerOutput;
    public float Efficiency        => _efficiency;

    // ── Initialisation ─────────────────────────────────────────────────
    private void Awake()
    {
        _temperature = ambientTemperature;
    }

    // ── Public API ─────────────────────────────────────────────────────

    /// <summary>Set the terminal voltage (positive = forward, negative = reverse).</summary>
    public void SetVoltage(float volts)
    {
        _voltage = volts;
    }

    /// <summary>Apply an external load torque opposing the shaft (N·m, always positive magnitude).</summary>
    public void SetLoadTorque(float loadNm)
    {
        _loadTorque = Mathf.Max(0f, loadNm);
    }

    /// <summary>
    /// Overrides the motor's internal angular velocity with a ground-truth value
    /// from an external source (e.g. a Rigidbody). Called by MotorShaftPhysics
    /// each FixedUpdate so back-EMF and current remain accurate when collisions
    /// or joints affect the shaft speed.
    /// </summary>
    public void OverrideAngularVelocity(float omega)
    {
        _angularVelocity = omega;
    }

    /// <summary>
    /// Overrides the motor's cumulative angle with the true angle integrated
    /// from the Rigidbody. Keeps the encoder reading the correct physical
    /// position rather than the motor's independently-simulated angle.
    /// </summary>
    public void OverrideAngle(float angleRad)
    {
        _angle = angleRad;
    }

    // ── Physics update ─────────────────────────────────────────────────
    private void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;

        // Guard against degenerate parameter values
        if (inductance <= 0f) inductance = 1e-6f;
        if (inertia    <= 0f) inertia    = 1e-9f;

        // ----------------------------------------------------------------
        // EXACT ZOH EXPONENTIAL INTEGRATION
        // For the ODE  dx/dt = a·x + b  the analytic solution over one step is:
        //
        //   x_new = x · e^(a·dt)  +  (b / a) · (e^(a·dt) - 1)       [a ≠ 0]
        //   x_new = x + b · dt                                         [a → 0, L'Hôpital]
        //
        // This is unconditionally stable regardless of dt, because e^(a·dt) → 0
        // as dt → ∞ whenever a < 0 (which is always true for physical motors).
        // ----------------------------------------------------------------

        // ── Step 1: Electrical subsystem ─────────────────────────────────
        // dI/dt = -(R/L)·I  +  (V - Ke·ω) / L
        //   a_elec = -R / L
        //   b_elec = (V - Ke·ω) / L   (treated as constant over the step)
        {
            float backEmf  = backEmfConstant * _angularVelocity;
            float a_elec   = -resistance / inductance;               // always < 0
            float b_elec   = (_voltage - backEmf) / inductance;
            float expElec  = Mathf.Exp(a_elec * dt);

            // Exact solution
            _current = _current * expElec + (b_elec / a_elec) * (expElec - 1f);
        }

        // ── Step 2: Friction & stiction ──────────────────────────────────
        float motorTorque  = torqueConstant * _current;
        float frictionSign = (_angularVelocity != 0f) ? Mathf.Sign(_angularVelocity)
                                                      : Mathf.Sign(motorTorque);

        // Stiction: if nearly stopped and torque is below static threshold, hold shaft
        bool stiction = Mathf.Abs(_angularVelocity) < 0.01f
                     && Mathf.Abs(motorTorque) <= staticFriction;

        // Coulomb friction opposes motion; viscous is handled inside the ODE below
        float coulombTorque = stiction ? motorTorque              // exactly cancels drive
                                       : frictionSign * staticFriction;

        // Load torque always opposes direction of motion
        float loadTorque = frictionSign * _loadTorque;

        // Constant forcing term for the mechanical ODE (everything except viscous drag)
        float mechForcing = motorTorque - coulombTorque - loadTorque;

        // ── Step 3: Mechanical subsystem ─────────────────────────────────
        // dω/dt = -(B/J)·ω  +  mechForcing / J
        //   a_mech = -B / J
        //   b_mech = mechForcing / J
        if (stiction)
        {
            // Shaft is held by stiction — zero velocity, zero acceleration
            _angularVelocity = 0f;
        }
        else
        {
            float a_mech = -viscousFriction / inertia;             // always ≤ 0

            if (Mathf.Abs(a_mech) > 1e-8f)
            {
                float b_mech  = mechForcing / inertia;
                float expMech = Mathf.Exp(a_mech * dt);

                // Exact ZOH solution for ω
                _angularVelocity = _angularVelocity * expMech
                                 + (b_mech / a_mech) * (expMech - 1f);

                // Exact integral of ω for θ:
                //   ∫₀^dt [ ω₀·e^(a·t) + (b/a)·(e^(a·t)-1) ] dt
                //   = (ω₀ - b/a) · (e^(a·dt)-1)/a  +  (b/a)·dt
                float omegaSteady = mechForcing / viscousFriction;  // b/a = -b_mech/a_mech
                _angle += ((_current * torqueConstant - coulombTorque - loadTorque) > 0f || _angularVelocity != 0f
                            ? (expMech - 1f) / a_mech * (_angularVelocity - omegaSteady)
                              + omegaSteady * dt
                            : 0f);
            }
            else
            {
                // Negligible viscous friction → pure constant acceleration (L'Hôpital)
                float alpha = mechForcing / inertia;
                _angle           += _angularVelocity * dt + 0.5f * alpha * dt * dt;
                _angularVelocity += alpha * dt;
            }
        }

        // Cache deliverable shaft torque
        _torque = motorTorque - coulombTorque;

        // ── Step 4: Thermal model ─────────────────────────────────────────
        // dT/dt = -(1/RC)·(T - T_amb)  +  I²R / C
        //   a_th = -1 / (R_th · C_th)
        //   b_th = (I²R + T_amb/R_th) / C_th   →  steady state at T_amb + I²R·R_th
        {
            float heatPower = _current * _current * resistance;   // I²R copper loss (W)
            float a_th      = -1f / (thermalResistance * thermalCapacitance);
            float b_th      = (heatPower + ambientTemperature / thermalResistance)
                            / thermalCapacitance;
            float expTh     = Mathf.Exp(a_th * dt);

            _temperature = _temperature * expTh + (b_th / a_th) * (expTh - 1f);
        }

        // ── Step 5: Performance metrics ───────────────────────────────────
        _powerInput  = _voltage * _current;
        _powerOutput = _torque  * _angularVelocity;
        _efficiency  = (_powerInput > 0.001f)
                     ? Mathf.Clamp01(_powerOutput / _powerInput)
                     : 0f;
    }

    // ── Debug Gizmo ────────────────────────────────────────────────────
    private void OnDrawGizmosSelected()
    {
        // Draw a small arc showing rotation direction and speed
        float arcLen = Mathf.Clamp(Mathf.Abs(_angularVelocity) * 0.05f, 0f, 2f);
        Gizmos.color = _angularVelocity >= 0f ? Color.green : Color.red;
        Vector3 p = transform.position;
        for (int i = 0; i < 20; i++)
        {
            float a0 = _angle + i       * arcLen / 20f;
            float a1 = _angle + (i + 1) * arcLen / 20f;
            Gizmos.DrawLine(
                p + new Vector3(Mathf.Cos(a0), 0, Mathf.Sin(a0)) * 0.3f,
                p + new Vector3(Mathf.Cos(a1), 0, Mathf.Sin(a1)) * 0.3f);
        }
    }
}
