using UnityEngine;

public class Robot : MonoBehaviour
{
    GameObject robotGO;
    [SerializeField] GameObject drivetrain;
    [SerializeField] RobotComponent[] robotComponents;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        robotGO = gameObject;
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    void setDrivetrainVelocity(Vector2 velocity)
    {

    }
}
