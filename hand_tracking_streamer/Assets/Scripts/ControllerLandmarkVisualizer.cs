using UnityEngine;

public class ControllerLandmarkVisualizer : MonoBehaviour
{
    private ControllerPoseStreamer _streamer;
    private GameObject _visual;

    public void Init(ControllerPoseStreamer streamer, GameObject prefab)
    {
        _streamer = streamer;
        if (prefab != null)
        {
            _visual = Instantiate(prefab, transform);
        }
    }

    private void Update()
    {
        if (_streamer == null || _visual == null) return;
        
        OVRInput.Controller controller = (_streamer.Side == ControllerPoseStreamer.ControllerSide.Left) 
            ? OVRInput.Controller.LTouch : OVRInput.Controller.RTouch;
            
        if ((OVRInput.GetConnectedControllers() & controller) != 0)
        {
            _visual.SetActive(true);
            // Find the tracking space center to convert local tracking to world space
            // Or simpler: just ensure the axes are rendered at the physical controller location
            transform.position = OVRManager.instance.GetComponentInChildren<OVRCameraRig>().trackingSpace.TransformPoint(OVRInput.GetLocalControllerPosition(controller));
            transform.rotation = OVRManager.instance.GetComponentInChildren<OVRCameraRig>().trackingSpace.rotation * OVRInput.GetLocalControllerRotation(controller);
        }
        else
        {
            _visual.SetActive(false);
        }
    }
}
