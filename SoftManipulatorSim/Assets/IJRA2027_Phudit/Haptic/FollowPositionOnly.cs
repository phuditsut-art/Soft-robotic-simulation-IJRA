using UnityEngine;

public class FollowPositionOnly : MonoBehaviour
{
    public Transform targetToFollow;

    [Header("Offsets")]
    [Tooltip("Adjust position offset relative to the target's local alignment.")]
    public Vector3 positionOffset = Vector3.zero;
    
    [Tooltip("Adjust rotation offset relative to the target's rotation.")]
    public Vector3 rotationOffset = Vector3.zero;

    void LateUpdate()
    {
        if (targetToFollow != null)
        {
            // 1. Calculate the target rotation with the offset applied
            Quaternion targetRotation = targetToFollow.rotation * Quaternion.Euler(rotationOffset);
            transform.rotation = targetRotation;

            // 2. Calculate the target position with the offset accounted for in the target's local space
            Vector3 targetPosition = targetToFollow.position + (targetToFollow.rotation * positionOffset);
            transform.position = targetPosition;
        }
    }
}