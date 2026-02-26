using UnityEngine;

public class Motor : MonoBehaviour
{
    [SerializeField] float startingPosition = 0;
    float position_ROTATIONS; //rotations
    float speed_RPS = 0; //rotations per second
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        position_ROTATIONS = startingPosition;
    }

    // Update is called once per frame
    void Update()
    {
        position_ROTATIONS += Time.deltaTime * speed_RPS;
    }

    public void setSpeed(float speed_RPS)
    {
        this.speed_RPS = speed_RPS;
    }

    public float getSpeed()
    {
        return speed_RPS;
    }

    public void setPosition(float position_ROTATIONS)
    {
        this.position_ROTATIONS = position_ROTATIONS;
    }

    public float getPosition()
    {
        return position_ROTATIONS;
    }
}
