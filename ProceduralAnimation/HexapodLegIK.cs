using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HexapodLegIK : MonoBehaviour
{
    public GameObject joint1;
    public GameObject joint2;
    public GameObject joint3;
    public GameObject endEffector;

    public GameObject debugSphere;

    public Vector3 legEndEffectorOffset = new Vector3(4f, 0, 0);

    public float legMovementSpeed = 10f;

    private bool legCanMove = false;

    public float a1 = 3;
    public float a2 = 3;
    public float a3 = 3;

    public float joint_1_limitAngleY;
    public float joint_1_limitAngleZ;
    public float joint_2_limitAngleZ;
    public float joint_3_limitAngleZ;

    public float legMaxDistance;

    public int legIndex;

    private bool legMoveFirst = true;

    private float radiusLength;
    private float initialDistanceXZ;
    private float initialDistanceY;
    private float initialEndEffectorY;
    private Vector3 targetEndEffectorPos;

    public GameObject body;

    public float bodySpeed;
    public float heightOffset;

    public bool rightLeg;

    private float theta1;
    private float theta2;
    private float theta3;
    private float theta4;

    public bool canBodyMove = true;

    /// <summary>
    /// TODO
    /// 
    /// cismin yuksekligi ayarlanacak
    /// 
    /// </summary>

    void Start()
    {
        UnityEditor.SceneView.FocusWindowIfItsOpen(typeof(UnityEditor.SceneView));

        ReleaseEndEffector();
        InitEndEffectorTransform();
    }

    void Update()
    {
        CalculateInverseKinematicsForLeg();
        RotateJoints();
        RaycastForTargetEndEffectorPosition();
        MoveLeg();
        CheckCanEndEffectorMove();
        BodyMovementControl();
        //body.transform.Translate(Vector3.forward * Time.deltaTime * bodySpeed);
        //body.transform.Translate(Vector3.left * Time.deltaTime * bodySpeed);
    }

    private void CalculateInverseKinematicsForLeg()
    {
        float X = endEffector.transform.position.x - joint1.transform.position.x;
        float Y = endEffector.transform.position.y - joint1.transform.position.y;
        float Z = endEffector.transform.position.z - joint1.transform.position.z;

        theta4 = Mathf.Atan(Z / X);
        float r_1 = Mathf.Sqrt(Mathf.Pow(Z, 2) + Mathf.Pow(X, 2));
        float r_2 = Mathf.Sqrt(Mathf.Pow(Mathf.Abs(Y) - a3, 2) + Mathf.Pow(r_1, 2));
        float fi_2 = Mathf.Acos((Mathf.Pow(a1, 2) + Mathf.Pow(r_2, 2) - Mathf.Pow(a2, 2)) / (2 * a1 * r_2));
        float fi_1 = Mathf.Atan(r_1 / (Mathf.Abs(Y) - a3));
        theta1 = fi_2 + fi_1 - Mathf.PI / 2;
        float fi_3 = Mathf.Acos((Mathf.Pow(a1, 2) + Mathf.Pow(a2, 2) - Mathf.Pow(r_2, 2)) / (2 * a1 * a2));
        theta2 = fi_3 - Mathf.PI;
        theta3 = Mathf.PI / 2 - fi_3 - theta1;
    }

    // In this function joint_1 y rotation remains same for the both legs because of calculation of theta_4
    // Atan(Z/X) probably does the job
    private void RotateJoints()
    {
        if (rightLeg)
        {
            joint1.transform.localRotation = Quaternion.Euler(-body.transform.eulerAngles.x, - theta4 * Mathf.Rad2Deg - body.transform.eulerAngles.y, theta1 * Mathf.Rad2Deg - body.transform.eulerAngles.z);
            joint2.transform.localRotation = Quaternion.Euler(0, 0, theta2 * Mathf.Rad2Deg);
            joint3.transform.localRotation = Quaternion.Euler(0, 0, theta3 * Mathf.Rad2Deg);
        }
        else
        {
            joint1.transform.localRotation = Quaternion.Euler(-body.transform.eulerAngles.x, - theta4 * Mathf.Rad2Deg - body.transform.eulerAngles.y, - theta1 * Mathf.Rad2Deg - body.transform.eulerAngles.z);
            joint2.transform.localRotation = Quaternion.Euler(0, 0, -theta2 * Mathf.Rad2Deg);
            joint3.transform.localRotation = Quaternion.Euler(0, 0, -theta3 * Mathf.Rad2Deg);
        }
    }

    private void InitEndEffectorTransform()
    {
        Vector3 pos = joint1.transform.position + legEndEffectorOffset;

        RaycastHit hitInfo;
        Ray landingRay = new Ray(pos, Vector3.down);

        if (Physics.Raycast(landingRay, out hitInfo))
        {
            if (hitInfo.collider.tag == "Environment")
                debugSphere.transform.position = hitInfo.point;
        }

        if (legIndex == 0 || legIndex == 1 || legIndex == 5)
            endEffector.transform.position = new Vector3(debugSphere.transform.position.x,
                debugSphere.transform.position.y, debugSphere.transform.position.z - legMaxDistance / 2);
        else
            endEffector.transform.position = debugSphere.transform.position;
    }

    private void RaycastForTargetEndEffectorPosition()
    {
        Vector3 pos = joint1.transform.position + legEndEffectorOffset;

        RaycastHit hitInfo;
        Ray landingRay = new Ray(pos, Vector3.down);

        if(Physics.Raycast(landingRay, out hitInfo))
        {
            if(hitInfo.collider.tag == "Environment")
                debugSphere.transform.position = hitInfo.point;
        }
    }

    private void MoveLeg()
    {
        if (legCanMove)
        {
            if (legMoveFirst)
            {
                radiusLength = Vector3.Magnitude(debugSphere.transform.position - endEffector.transform.position) / 2;
                initialDistanceXZ = GetDistanceInXZPlane(debugSphere.transform.position, endEffector.transform.position);
                initialDistanceY = debugSphere.transform.position.y - endEffector.transform.position.y;
                initialEndEffectorY = endEffector.transform.position.y;
                targetEndEffectorPos = debugSphere.transform.position;
                legMoveFirst = false;
            }

            float currentDistanceXZ = GetDistanceInXZPlane(targetEndEffectorPos, endEffector.transform.position);

            float percentage = 1 - currentDistanceXZ / initialDistanceXZ;

            float y = Mathf.Sin(percentage * Mathf.PI) * radiusLength * 0.5f + initialEndEffectorY + initialDistanceY;

            endEffector.transform.position = new Vector3(endEffector.transform.position.x, y, endEffector.transform.position.z);
            Vector3 actualTargetPosition = new Vector3(targetEndEffectorPos.x, endEffector.transform.position.y, targetEndEffectorPos.z);

            float step = legMovementSpeed * Time.deltaTime;
            endEffector.transform.position = Vector3.MoveTowards(endEffector.transform.position, actualTargetPosition, step);

            if (Vector3.Magnitude(targetEndEffectorPos - endEffector.transform.position) <= 0.01f)
            {
                legCanMove = false;
                legMoveFirst = true;
            }
        }
    }

    private void ReleaseEndEffector()
    {
        endEffector.transform.parent = null;
    }

    private void CheckCanEndEffectorMove()
    {
        if (rightLeg)
        {
            if (Vector3.Magnitude(debugSphere.transform.position - endEffector.transform.position) >= legMaxDistance ||
            joint2.transform.localRotation.z * Mathf.Rad2Deg >= -joint_2_limitAngleZ || joint3.transform.localRotation.z * Mathf.Rad2Deg >= -joint_3_limitAngleZ)
            {
                legCanMove = true;
                canBodyMove = false;
            }
            else
                canBodyMove = true;

        }
        else
        {
            if (Vector3.Magnitude(debugSphere.transform.position - endEffector.transform.position) >= legMaxDistance ||
            joint2.transform.localRotation.z * Mathf.Rad2Deg < joint_2_limitAngleZ || joint3.transform.localRotation.z * Mathf.Rad2Deg < joint_3_limitAngleZ)
            {
                legCanMove = true;
                canBodyMove = false;
            }
            else
                canBodyMove = true;
        }

        //if (rightLeg)
        //{
        //    if (Vector3.Magnitude(debugSphere.transform.position - endEffector.transform.position) >= legMaxDistance)
        //        legCanMove = true;
        //}
        //else
        //{
        //    if (Vector3.Magnitude(debugSphere.transform.position - endEffector.transform.position) >= legMaxDistance)
        //        legCanMove = true;
        //}
    }

    private void BodyMovementControl()
    {
        //if(!canBodyMove)
        //    body.transform.GetComponent<Rigidbody>().velocity = Vector3.zero;
    }

    private void FixBodyHeight()
    {
        if((joint1.transform.position.y - joint3.transform.position.y) < heightOffset)
        {
            float distance = heightOffset - (joint1.transform.position.y - joint3.transform.position.y);
            body.transform.position = new Vector3(body.transform.position.x, body.transform.position.y + distance, body.transform.position.z);
        }
    }

    private float GetDistanceInXZPlane(Vector3 point1, Vector3 point2)
    {
        return Mathf.Sqrt(Mathf.Pow(point2.x - point1.x, 2) + Mathf.Pow(point2.z - point1.z, 2));
    }

    private void OnDrawGizmosSelected()
    {
        //Gizmos.DrawLine(targetEndEffectorPos, endEffector.transform.position);
    }

}
