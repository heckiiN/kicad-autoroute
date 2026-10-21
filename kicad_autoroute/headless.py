"""Imported before pcbnew by every script that runs in KiCad's python. KiCad reports some problems
(a font it substitutes, a failed assert) in a message box; without anyone to press OK that blocks a
run for good. Those messages go to stderr instead."""
try:
    import wx

    wx.Log.SetActiveTarget(wx.LogStderr())
    wx.DisableAsserts()
except Exception:   # no wx (not KiCad's python): nothing to redirect
    pass
