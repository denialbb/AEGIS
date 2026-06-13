; ============================================================
;  AEGIS Review Trigger — AutoHotkey v2
;  Bridges the other agent's output to a Claude review session.
;
;  TWO MODES (both active simultaneously):
;    1. File Watcher  — polls for REVIEW_REQUESTED.flag every N seconds
;                       the other agent drops this file when ready
;    2. Manual Hotkey — Win+Shift+R to trigger immediately
;
;  SETUP:
;    1. Set SHARED_DIR to your actual shared folder path
;    2. Set CLAUDE_URL if using a specific project URL
;    3. Run this script (it sits in the tray silently)
; ============================================================

#Requires AutoHotkey v2.0
#SingleInstance Force
SetWorkingDir A_ScriptDir


; ── CONFIGURATION ───────────────────────────────────────────

SHARED_DIR    := "C:\Projects\AEGIS\.agents\shared\"
PENDING_FILE  := SHARED_DIR . "queue\PENDING_REVIEW.md"
FLAG_FILE     := SHARED_DIR . "queue\REVIEW_REQUESTED.flag"
POLL_INTERVAL := 5000   ; milliseconds between file watcher checks
CLAUDE_URL    := "https://claude.ai/new"               ; or your project URL


; ── SYSTEM TRAY SETUP ───────────────────────────────────────

TraySetIcon "shell32.dll", 44   ; satellite dish icon
A_IconTip := "AEGIS Review Trigger — Active"
tray := A_TrayMenu
tray.Delete()
tray.Add("AEGIS Review Trigger", (*) => "")
tray.Disable("AEGIS Review Trigger")
tray.Add()
tray.Add("Trigger Review Now`tWin+Shift+R", (*) => TriggerReview())
tray.Add("Open Shared Folder", (*) => Run("explorer.exe " . SHARED_DIR))
tray.Add()
tray.Add("Exit", (*) => ExitApp())


; ── FILE WATCHER LOOP ────────────────────────────────────────

SetTimer(WatchForFlag, POLL_INTERVAL)

WatchForFlag() {
    global FLAG_FILE
    if FileExist(FLAG_FILE) {
        FileDelete(FLAG_FILE)    ; consume the flag immediately
        TriggerReview()
    }
}


; ── MANUAL HOTKEY ────────────────────────────────────────────

; Win + Shift + R
#+r:: TriggerReview()


; ── CORE FUNCTION ────────────────────────────────────────────

TriggerReview() {
    global PENDING_FILE, CLAUDE_URL

    ; Validate the pending review file exists
    if !FileExist(PENDING_FILE) {
        MsgBox(
            "PENDING_REVIEW.md not found.`n`nExpected at:`n" . PENDING_FILE,
            "AEGIS — No Pending Review",
            "Icon!"
        )
        return
    }

    ; Read the file
    pendingContent := FileRead(PENDING_FILE, "UTF-8")

    if StrLen(Trim(pendingContent)) < 50 {
        MsgBox(
            "PENDING_REVIEW.md appears empty or incomplete.`n`nThe other agent should fill it out before triggering a review.",
            "AEGIS — Incomplete Submission",
            "Icon!"
        )
        return
    }

    ; Build the prompt
    prompt := BuildPrompt(pendingContent)

    ; Put it on the clipboard
    A_Clipboard := ""
    A_Clipboard := prompt
    if !ClipWait(3) {
        MsgBox("Clipboard operation timed out.", "AEGIS — Error", "Icon!")
        return
    }

    ; Open Claude in the default browser
    Run(CLAUDE_URL)

    ; Wait for the page and input box to be ready
    ; Adjust Sleep duration if your machine is slower/faster
    Sleep(4000)

    ; Paste into the chat input
    Send("^v")

    ; Brief confirmation in tray
    TrayTip("Review prompt loaded into Claude.`nPress Enter to send.", "AEGIS", 1)
    ; Note: intentionally NOT auto-sending — human stays in the loop
}


; ── PROMPT BUILDER ───────────────────────────────────────────

BuildPrompt(pendingContent) {
    return "
    (
    ## AEGIS CODE REVIEW REQUEST

    Please do the following:
    1. Read the PENDING_REVIEW.md content below carefully.
    2. Read the referenced changed files from the shared directory / project context.
    3. Produce a structured REVIEW_[timestamp].md using the established review template.
    4. Flag all BLOCKERS, MAJORS, and MINORS with the severity conventions we use.
    5. Fill in the module-specific check tables honestly.

    You have access to the project files. Focus on:
    - Mathematical correctness (Kalman filter, pseudo-inverse, FDI thresholds)
    - Type safety and mypy compliance
    - State machine transition validity
    - Module boundary integrity (decoupling)
    - Numerical edge cases (singular matrices, all engines dead, single survivor)

    ---

    )" . pendingContent
}
