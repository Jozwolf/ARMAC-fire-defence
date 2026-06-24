# iPad → DEVONthink: Multi-Page Document Capture

Four methods depending on what you are capturing and where you are in the workflow.

---

## Method 1 – Safari web pages (easiest, no extra tools)

1. Open the page in Safari.
2. Take a screenshot (Side button + Volume Up).
3. Tap the **preview thumbnail** in the bottom-left corner.
4. Tap **"Full Page"** at the top of the editor.
   - Safari renders the entire page as a single tall PDF automatically.
5. Tap the **Share** button → **DEVONthink To Go** → choose your group → **Import**.

This works for any website and requires nothing extra.

---

## Method 2 – Markup screenshot direct to a DTTG group (recommended for New Scientist etc.)

This method replaces the cumbersome route of Markup → Save to Files → navigate to Files → share to DTTG. Instead, you share directly from inside Markup and the file lands in the correct DTTG group in one step.

### How it works

DEVONthink To Go provides its own **Shortcuts actions**, including "Create Item in DEVONthink To Go", which can receive a file and place it into a named database and group — no DTTG interface interaction required. A Shortcut placed in the iOS Share Sheet becomes available as a one-tap action from anywhere: Markup, Files, Photos, Safari, or any other app.

### One-time setup: create the Shortcut

Open the **Shortcuts** app on your iPad and create a new Shortcut named **"File to DTTG"**.

| Step | Action | Setting |
|------|--------|---------|
| 1 | **Receive** input | Type: Any · From: Share Sheet and Shortcuts |
| 2 | **Choose from Menu** | Prompt: "File to which group?" |
| 3 | ↳ Menu item: **"New Scientist"** | — |
| 4 | ↳↳ **Create Item in DEVONthink To Go** | Item: Shortcut Input · Database: (your database) · Group: New Scientist |
| 5 | ↳ Menu item: **"Fire Defence"** | — |
| 6 | ↳↳ **Create Item in DEVONthink To Go** | Item: Shortcut Input · Database: (your database) · Group: Fire Defence |
| 7 | ↳ Menu item: **"Inbox"** | — |
| 8 | ↳↳ **Create Item in DEVONthink To Go** | Item: Shortcut Input · Database: (your database) · Group: Inbox |

Add or remove menu items to match your actual DTTG groups. The "Create Item in DEVONthink To Go" action is found by searching **"DEVONthink"** in the Shortcuts action library — it only appears after DTTG is installed.

### Using it from Markup

1. Take a screenshot (Side button + Volume Up) — or open an existing image in Markup.
2. Annotate as needed in Markup.
3. Tap the **Share button** (arrow-out-of-box, top right) — **not** Done → Save to Files.
4. In the share sheet, tap **"File to DTTG"** (your Shortcut).
5. Tap the group name you want.
6. The file appears in that DTTG group immediately. Syncs to DEVONthink on your Mac via the Contabo syncstore.

### Using it from Files (for files already saved there)

1. Open **Files**, navigate to the saved file.
2. Long-press the file → **Share**.
3. Tap **"File to DTTG"** in the share sheet.
4. Choose the group.

### Notes

- The Shortcut works for any file type: PNG, JPG, PDF, even text files.
- If "Create Item in DEVONthink To Go" does not appear in the Shortcuts action library, check that DTTG is installed and open Settings → DEVONthink To Go → allow Siri & Shortcuts.
- The file goes straight into the group — DTTG does not need to be open or in the foreground.

---

## Method 3 – iOS Shortcut for multi-page scrolling documents

Use this when the document is longer than one screen in an app that does not support Full Page export (not Safari).

### Shortcut steps

| Step | Action | Setting |
|------|--------|---------|
| 1 | **Ask for Input** | Prompt: "Document title?" · Input type: Text |
| 2 | Set variable | **doc_title** ← Shortcut Input |
| 3 | **Repeat** | (loop indefinitely) |
| 4 | ↳ Take Screenshot | — |
| 5 | ↳ Add to Variable | Variable: **screenshots** |
| 6 | ↳ **Choose from Menu** | Prompt: "More pages?" · Options: "Yes, scroll & continue" / "No, done" |
| 7 | ↳ If "No, done" → **Exit Repeat** | — |
| 8 | **Combine Images** | Images: **screenshots** · Layout: Vertically |
| 9 | **Make PDF** | Input: combined image · Page size: actual size |
| 10 | **Choose from Menu** | Prompt: "File to which group?" (same group menu as Method 2) |
| 11 | ↳ **Create Item in DEVONthink To Go** | Item: PDF from step 9 · Group: chosen group |

### How to use it

1. Navigate to the first screen of the document.
2. Run the Shortcut from your Home Screen or the Shortcuts widget.
3. Enter a title.
4. Take screenshot, scroll, tap **"Yes"**, repeat for each screen.
5. On the last screen tap **"No, done"**.
6. Choose the DTTG group — the stitched PDF is filed immediately.

---

## Method 4 – Automatic Mac-side stitching (best image quality, optional)

Use this only when you need the overlap-trimming that the iOS Shortcut cannot provide — for example, very long documents where duplicate scroll regions are visible in the stitched result.

### Setup (one time, on your Mac)

```bash
pip3 install Pillow numpy watchdog
```

### Watch mode

```bash
python3 watch_and_import.py \
    ~/Library/Mobile\ Documents/com~apple~CloudDocs/Screenshots \
    --group "Fire Defence"
```

Screenshots synced via iCloud are automatically stitched, trimmed, and imported into DEVONthink on the Mac. DEVONthink then syncs the result back to DTTG via the Contabo syncstore.

### One-off manual stitch

```bash
python3 stitch_to_devonthink.py page1.png page2.png page3.png \
    --import --group "Fire Defence"
```

---

## Choosing the right method

| Situation | Method |
|-----------|--------|
| Web page in Safari | 1 – Full Page screenshot |
| Markup screenshot or any single-page file | 2 – "File to DTTG" Shortcut via Share button |
| File already saved in the Files app | 2 – same Shortcut, from Files share sheet |
| Multi-page scrolling document, any app | 3 – screenshot loop Shortcut |
| Very long document, overlap trimming needed | 4 – Mac script |
