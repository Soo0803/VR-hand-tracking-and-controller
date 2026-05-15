using UnityEngine;

/// <summary>
/// Professional diagnostic component that anchors a GameObject to the Head (Main Camera).
/// Ensures that visual data for a specific side is always in the correct L/R position
/// relative to the user's view.
/// </summary>
public class SideAnchor : MonoBehaviour
{
    public enum Side { Left, Right }

    [SerializeField] private Side side;
    
    // Default offsets for a comfortable diagnostic view
    // -0.3/+0.3 X (Horizontal), -0.2 Y (Below eye level), 0.5 Z (In front of face)
    [SerializeField] private Vector3 offset = new Vector3(0.3f, -0.2f, 0.5f);

    private Transform _headTransform;

    public void SetSide(Side newSide) => side = newSide;

    private void Start()
    {
        FindHead();
    }

    private void LateUpdate()
    {
        if (_headTransform == null)
        {
            FindHead();
            if (_headTransform == null) return;
        }

        // Apply horizontal flip if it's the Left side
        float xOffset = (side == Side.Left) ? -offset.x : offset.x;
        Vector3 finalRelativePos = new Vector3(xOffset, offset.y, offset.z);

        // Position diagnostic relative to head rotation/position
        transform.position = _headTransform.position + (_headTransform.rotation * finalRelativePos);
        transform.rotation = _headTransform.rotation;
        
        // Apply mirror effect for easier human verification
        // (Diagnostic data flips so L-is-L and R-is-R in user field of view)
        transform.localScale = new Vector3(side == Side.Left ? -1 : 1, 1, 1);
    }

    private void FindHead()
    {
        if (Camera.main != null)
        {
            _headTransform = Camera.main.transform;
        }
        else
        {
            // Fallback for OVR rig if Camera.main isn't tagged
            GameObject centerAnchor = GameObject.Find("CenterEyeAnchor");
            if (centerAnchor != null) _headTransform = centerAnchor.transform;
        }
    }
}
