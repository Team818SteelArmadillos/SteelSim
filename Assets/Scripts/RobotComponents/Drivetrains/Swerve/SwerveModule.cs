using UnityEngine;

public class SwerveModule : MonoBehaviour
{
    GameObject wheel;
    Rigidbody wheelRB;
    //Motor steerMotor = new KrakenX60();
    //Motor driveMotor = new KrakenX60();

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        wheel = gameObject;
        wheelRB = gameObject.GetComponent<Rigidbody>();
        //driveMotor.setSpeed(0.5f);
    }

    // Update is called once per frame
    void Update()
    {
        //Vector3 angularVelocity = new Vector3(0f,0f,0f);
        //angularVelocity.y = rotationToDegrees(steerMotor.getSpeed());
        //angularVelocity.z = rotationToDegrees(driveMotor.getSpeed());
        //wheelRB.angularVelocity = angularVelocity;

    }

    private float rotationToDegrees(float rotation) 
    {
        return rotation * 360f;
    }
}
