using UnityEngine;

/// <summary>
/// Visualizes a Quest controller's pose in the VR scene using an axis gizmo prefab.
/// Shows a single axis marker at the controller position + rotation when streaming
/// and "Show Landmarks" is enabled. Auto-created at runtime by ControllerPoseStreamer.
/// </summary>
public class ControllerLandmarkVisualizer : MonoBehaviour
{
    [SerializeField] private GameObject _axisPrefab;
    [SerializeField] private float _scale = 0.04f;

    private ControllerPoseStreamer _streamer;
    private GameObject _axisGizmo;
    private bool _created = false;

    /// <summary>
    /// Initialise the visualizer at runtime. Called by ControllerPoseStreamer.
    /// </summary>
    public void Init(ControllerPoseStreamer streamer, GameObject axisPrefab)
    {
        _streamer = streamer;
        _axisPrefab = axisPrefab;
        CreateGizmo();
    }

    private void CreateGizmo()
    {
        if (_axisPrefab == null) return;
        _axisGizmo = Instantiate(_axisPrefab, transform);
        _axisGizmo.transform.localScale = Vector3.one * _scale;
        _axisGizmo.SetActive(false);
        _created = true;
    }

    private void Update()
    {
        if (!_created || _axisGizmo == null) return;

        // Gate: streaming + Show Landmarks + controller-enabled mode
        if (AppManager.Instance == null
            || !AppManager.Instance.isStreaming
            || !AppManager.Instance.ShowLandmarks
            || !AppManager.Instance.TrackControllers)
        {
            if (_axisGizmo.activeSelf) _axisGizmo.SetActive(false);
            return;
        }

        // Get current controller pose from OVRInput
        OVRInput.Controller ctrl = _streamer.Side == ControllerPoseStreamer.ControllerSide.Left
            ? OVRInput.Controller.LTouch
            : OVRInput.Controller.RTouch;

        bool isTracked = OVRInput.GetControllerPositionTracked(ctrl)
                      && OVRInput.GetControllerOrientationTracked(ctrl);

        if (!isTracked)
        {
            if (_axisGizmo.activeSelf) _axisGizmo.SetActive(false);
            return;
        }

        Vector3 position = OVRInput.GetLocalControllerPosition(ctrl);
        Quaternion rotation = OVRInput.GetLocalControllerRotation(ctrl);

        _axisGizmo.SetActive(true);
        _axisGizmo.transform.SetPositionAndRotation(position, rotation);
    }

    private void OnDestroy()
    {
        if (_axisGizmo != null) Destroy(_axisGizmo);
    }
}
