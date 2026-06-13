; ============================================================
;  AEGIS Review Trigger — AutoHotkey v2
;  Bridges the other agent's output to a Claude desktop app.
;
;  TWO MODES (both active simultaneously):
;    1. File Watcher  — polls for REVIEW_REQUESTED.flag every N seconds
;                       the other agent drops this file when ready
;    2. Manual Hotkey — Win+Shift+R to trigger immediately
;
;  SETUP:
;    1. Set SHARED_DIR to your actual shared folder path
;    2. Run this script (it sits in the tray silently)
;    3. If the app isn't found, run it once manually so AHK can see it
; ============================================================

#Requires AutoHotkey v2.0
#SingleInstance Force
SetWorkingDir A_ScriptDir

; Per-Monitor DPI awareness — without this, AHK can misjudge window
; positions/sizes on multi-monitor setups where displays use different
; scaling factors, causing clicks to land on the wrong monitor or edge.
DllCall("SetThreadDpiAwarenessContext", "ptr", -4, "ptr")


; ── CONFIGURATION ───────────────────────────────────────────

SHARED_DIR     := "C:\Projects\AEGIS\.agents\shared\"
PENDING_FILE   := SHARED_DIR . "queue\PENDING_REVIEW.md"
TEMPLATE_FILE  := SHARED_DIR . "reviews\REVIEW_template.md"
FLAG_FILE      := SHARED_DIR . "queue\REVIEW_REQUESTED.flag"
POLL_INTERVAL  := 10000  ; milliseconds between file watcher checks

; Claude desktop app identifiers — try exe first, fall back to title
CLAUDE_EXE    := "claude.exe"
CLAUDE_TITLE  := "Claude"                              ; partial window title match


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
    global PENDING_FILE, TEMPLATE_FILE, CLAUDE_EXE, CLAUDE_TITLE

    ; ── 1. Validate the pending review file ─────────────────
    if !FileExist(PENDING_FILE) {
        MsgBox(
            "PENDING_REVIEW.md not found.`n`nExpected at:`n" . PENDING_FILE,
            "AEGIS — No Pending Review",
            "Icon!"
        )
        return
    }

    pendingContent := FileRead(PENDING_FILE, "UTF-8")

    if StrLen(Trim(pendingContent)) < 50 {
        MsgBox(
            "PENDING_REVIEW.md appears empty or incomplete.`n`nThe other agent should fill it out before triggering a review.",
            "AEGIS — Incomplete Submission",
            "Icon!"
        )
        return
    }

    ; ── 2. Load the review template ─────────────────────────
    if !FileExist(TEMPLATE_FILE) {
        MsgBox(
            "REVIEW_template.md not found.`n`nExpected at:`n" . TEMPLATE_FILE,
            "AEGIS — Missing Template",
            "Icon!"
        )
        return
    }

    templateContent := FileRead(TEMPLATE_FILE, "UTF-8")

    ; ── 3. Find the Claude desktop window ───────────────────
    claudeHwnd := WinExist("ahk_exe " . CLAUDE_EXE)
    if !claudeHwnd
        claudeHwnd := WinExist(CLAUDE_TITLE . " ahk_class Chrome_WidgetWin_1")
    if !claudeHwnd
        claudeHwnd := WinExist(CLAUDE_TITLE)

    if !claudeHwnd {
        MsgBox(
            "Claude desktop app not found.`n`nMake sure Claude is open and visible before triggering a review.",
            "AEGIS — Claude Not Found",
            "Icon!"
        )
        return
    }

    ; ── 4. Bring Claude to the foreground ───────────────────
    WinRestore(claudeHwnd)
    WinActivate(claudeHwnd)
    if !WinWaitActive(claudeHwnd, , 3) {
        MsgBox("Could not focus the Claude window.", "AEGIS — Error", "Icon!")
        return
    }

    ; ── 5. Click center of window to clear any stray text field focus ────
    ; If the caret is in a text input, Tab won't navigate correctly.
    ; We click relative to the active window's CLIENT AREA (not absolute
    ; screen coordinates) — this avoids issues on dual-monitor setups where
    ; differing DPI scaling or monitor placement can throw off screen math
    ; and send the click to the wrong monitor or off-screen.
    WinGetClientPos(, , &width, &height, claudeHwnd)
    prevCoordMode := A_CoordModeMouse
    CoordMode("Mouse", "Client")
    MouseClick("left", width // 2, height // 2)
    CoordMode("Mouse", prevCoordMode)
    Sleep(200)

    ; ── 6. Open a new project chat ──────────────────────────
    ; Tab moves focus to the new-chat button; Enter activates it.
    ; Ctrl+N opens a new chat outside the project — don't use it.
    Send("{Tab}{Enter}")
    Sleep(500)

    ; ── 7. Build prompt and paste ───────────────────────────
    prompt := BuildPrompt(templateContent, pendingContent)

    A_Clipboard := ""
    A_Clipboard := prompt
    if !ClipWait(3) {
        MsgBox("Clipboard operation timed out.", "AEGIS — Error", "Icon!")
        return
    }

    Send("^v")

    ; ── 8. Confirm ──────────────────────────────────────────
    TrayTip("Review prompt loaded.`nPress Enter to send.", "AEGIS", 1)
    ; Intentionally NOT auto-sending — human stays in the loop
}


; ── PROMPT BUILDER ───────────────────────────────────────────

BuildPrompt(templateContent, pendingContent) {
    header := "
    (
## AEGIS CODE REVIEW REQUEST

A submission is ready for your review. Execute the following:
1. Read the submission (Section B) carefully, including all self-identified concerns.
2. Read the referenced changed files from the project context.
3. Produce a filled review document using the template in Section A below.
4. Save it as REVIEW_[YYYY-MM-DD_HHMM].md in the shared reviews folder.

---

## SECTION A — OUTPUT TEMPLATE

    )"

    bridge := "
    (

---

## SECTION B — SUBMISSION

    )"

    return header . templateContent . bridge . pendingContent
}
