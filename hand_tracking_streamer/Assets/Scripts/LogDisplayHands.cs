using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Text;
using System.Collections.Generic;

public class LogDisplayHands : MonoBehaviour
{
    [SerializeField] private TMP_Text logText;
    [SerializeField] private Text legacyLogText;

    [SerializeField] private string logSource; 

    // Optimization: Cache StringBuilder to avoid memory garbage
    private StringBuilder _sb = new StringBuilder(1000);

    private void Update()
    {
        DisplayLog();
    }

    private void DisplayLog()
    {
        if (LogManager.Instance == null ||
            (AppManager.Instance != null && !AppManager.Instance.ShowDebugInfo))
        {
            SetText(string.Empty);
            return;
        }

        // Use slot-based messages for deterministic ordering
        var messages = LogManager.Instance.GetSlotMessages(logSource);

        if (messages == null || messages.Count == 0)
        {
            SetText(string.Empty);
            return;
        }

        _sb.Clear();
        for (int i = 0; i < messages.Count; i++)
        {
            _sb.AppendLine(messages[i]);
        }

        SetText(_sb.ToString());
    }

    private void SetText(string text)
    {
        if (logText != null)
        {
            logText.text = text;
        }
        if (legacyLogText != null)
        {
            legacyLogText.text = text;
        }
    }
}
