using UnityEngine;
using System.Collections.Generic;

public class LogManager : MonoBehaviour
{
    public static LogManager Instance { get; private set; }

    // ── Slot-based logging ──
    // Each source (e.g. "Left", "Right") has keyed slots that are OVERWRITTEN each frame.
    // This gives stable, deterministic ordering in the display.
    // Keys: "wrist", "landmarks", "controller", "head", "status"
    private Dictionary<string, Dictionary<string, string>> _slots
        = new Dictionary<string, Dictionary<string, string>>();

    // Define the fixed display order of slot keys
    private static readonly string[] _displayOrder = {
        "status",       // connection / streaming status
        "controller",   // controller pose data
        "wrist",        // hand wrist data
        "landmarks",    // hand landmarks
        "head"          // head pose
    };

    // Legacy append-based log (still used by general status messages)
    private Dictionary<string, List<string>> logMessages = new Dictionary<string, List<string>>();

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(gameObject);
        }
    }

    /// <summary>
    /// Log a message to a specific source using a keyed slot (overwrites previous value).
    /// This provides stable display ordering across frames.
    /// </summary>
    public void LogSlot(string source, string slotKey, string message)
    {
        if (!_slots.ContainsKey(source))
        {
            _slots[source] = new Dictionary<string, string>();
        }
        _slots[source][slotKey] = message;
        Debug.Log($"[{source}/{slotKey}] {message}");
    }

    /// <summary>
    /// Get all slot messages for a source in the fixed display order.
    /// Returns only slots that have content.
    /// </summary>
    public List<string> GetSlotMessages(string source)
    {
        var result = new List<string>();
        if (!_slots.ContainsKey(source)) return result;

        var sourceSlots = _slots[source];
        foreach (string key in _displayOrder)
        {
            if (sourceSlots.ContainsKey(key) && !string.IsNullOrEmpty(sourceSlots[key]))
            {
                result.Add(sourceSlots[key]);
            }
        }
        return result;
    }

    /// <summary>
    /// Clear all slots for a source (call when streaming stops).
    /// </summary>
    public void ClearSlots(string source)
    {
        if (_slots.ContainsKey(source))
        {
            _slots[source].Clear();
        }
    }

    /// <summary>
    /// Clear all slots for all sources.
    /// </summary>
    public void ClearAllSlots()
    {
        foreach (var kvp in _slots)
        {
            kvp.Value.Clear();
        }
    }

    // ── Legacy API (kept for backward compatibility with status messages) ──

    // Log a message to a specific source (append-based)
    public void Log(string source, string message)
    {
        if (!logMessages.ContainsKey(source))
        {
            logMessages[source] = new List<string>();
        }
        logMessages[source].Add(message);
        Debug.Log($"[{source}] {message}");
    }

    // Get log messages for a specific source (legacy)
    public List<string> GetLogMessages(string source)
    {
        if (logMessages.ContainsKey(source))
        {
            return logMessages[source];
        }
        return new List<string>();
    }
}
