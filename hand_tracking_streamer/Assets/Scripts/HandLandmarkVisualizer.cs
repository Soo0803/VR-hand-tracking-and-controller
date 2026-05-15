using UnityEngine;
using Oculus.Interaction.Input;

public class HandLandmarkVisualizer : MonoBehaviour
{
    [SerializeField] private HandLandmarkStreamer _streamer;
    [SerializeField] private GameObject _axisPrefab;
    [SerializeField] private float _scale = 0.02f;

    private GameObject[] _visualizerPool;
    private bool _poolCreated = false;

    // The same 21 joints used in your streamer
    private readonly int[] _jointsToTrack = {
        1, 2, 3, 4, 5, 7, 8, 9, 10, 12, 13, 14, 15, 17, 18, 19, 20, 22, 23, 24, 25 
    };

    private void Start()
    {
        if (_streamer == null) _streamer = GetComponent<HandLandmarkStreamer>();
        CreatePool();
    }

    private void CreatePool()
    {
        _visualizerPool = new GameObject[_jointsToTrack.Length];
        for (int i = 0; i < _jointsToTrack.Length; i++)
        {
            _visualizerPool[i] = Instantiate(_axisPrefab, transform);
            _visualizerPool[i].transform.localScale = Vector3.one * _scale;
            _visualizerPool[i].SetActive(false);
        }
        _poolCreated = true;
    }

    private void Update()
    {
        if (!AppManager.Instance.isStreaming || !AppManager.Instance.ShowLandmarks)
        {
            ToggleAllVisualizers(false);
            return;
        }

        // Check if this hand should even be active based on AppManager selection
        int mode = AppManager.Instance.SelectedHandMode;
        if ((mode == 1 && _streamer.Side == HandLandmarkStreamer.HandSide.Right) ||
            (mode == 2 && _streamer.Side == HandLandmarkStreamer.HandSide.Left))
        {
            ToggleAllVisualizers(false);
            return;
        }

        UpdateVisuals();
    }

    private void UpdateVisuals()
    {
        IHand hand = _streamer.Hand;
        if (hand == null || !hand.IsTrackedDataValid)
        {
            ToggleAllVisualizers(false);
            return;
        }

        if (hand.GetRootPose(out Pose rootPose) && 
            hand.GetJointPosesFromWrist(out ReadOnlyHandJointPoses joints))
        {
            // Position this root object at the wrist's tracking location in WORLD SPACE
            // This ensures landmarks follow the hand as it moves.
            transform.position = rootPose.position;
            transform.rotation = rootPose.rotation;

            for (int i = 0; i < _jointsToTrack.Length; i++)
            {
                int jointIndex = _jointsToTrack[i];
                if (jointIndex < joints.Count)
                {
                    _visualizerPool[i].SetActive(true);
                    
                    // joints[index] is already relative to the wrist (rootPose)
                    _visualizerPool[i].transform.localPosition = joints[jointIndex].position;
                    _visualizerPool[i].transform.localRotation = joints[jointIndex].rotation;
                }
            }
        }
    }

    private void ToggleAllVisualizers(bool state)
    {
        if (!_poolCreated) return;
        foreach (var obj in _visualizerPool)
        {
            if (obj.activeSelf != state) obj.SetActive(state);
        }
    }
}