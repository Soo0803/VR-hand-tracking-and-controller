using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Text;

public class LogDisplayNetwork : MonoBehaviour
{
    [SerializeField] private TMP_Text logText;
    [SerializeField] private Text legacyLogText;
    [SerializeField] private int maxDisplayedMessages = 20;
    // New: the source of logs this display should show
    [SerializeField] private string logSource;
    private readonly StringBuilder _sb = new StringBuilder(1024);

    private void Update()
    {
        DisplayLog();
    }

    private void DisplayLog()
    {
        if (LogManager.Instance == null)
        {
            SetText(string.Empty);
            return;
        }

        var legacyMessages = LogManager.Instance.GetLogMessages(logSource);
        var slotMessages = LogManager.Instance.GetSlotMessages(logSource);

        _sb.Clear();
        
        int startIdx = Mathf.Max(0, legacyMessages.Count - maxDisplayedMessages);
        for (int i = startIdx; i < legacyMessages.Count; i++)
        {
            _sb.AppendLine(legacyMessages[i]);
        }

        for (int i = 0; i < slotMessages.Count; i++)
        {
            _sb.AppendLine(slotMessages[i]);
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
