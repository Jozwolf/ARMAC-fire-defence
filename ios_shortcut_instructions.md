# iPad → DEVONthink: Multi-Page Document Capture

Two complementary methods depending on what you are looking at on the iPad.

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

## Method 2 – iOS Shortcut for any app (PDFs, emails, documents, maps…)

Create the following Shortcut in the **Shortcuts** app on your iPad.

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
| 10 | **Share** PDF | Share with: **DEVONthink To Go** |

### How to use it

1. Navigate to the first page of the document on your iPad.
2. Run the Shortcut (add it to your Home Screen or the Share Sheet for one tap).
3. Enter a title when prompted.
4. Take screenshot, scroll down, tap **"Yes"**, repeat for each page.
5. When on the last page, tap **"No, done"**.
6. DEVONthink To Go opens automatically with the finished PDF ready to file.

---

## Method 3 – Automatic Mac-side stitching (best image quality)

Use this when you want overlap-trimming and higher-quality output.

### Setup (one time, on your Mac)

```bash
# Install dependencies
pip3 install Pillow numpy watchdog

# Clone / open the project folder
cd ~/path/to/ARMAC-fire-defence
```

### Configure iCloud sync

On your iPad, enable **iCloud Drive** for the **Screenshots** album  
(Settings → [Your Name] → iCloud → Photos → turn on iCloud Photos).  
On your Mac, the screenshots appear at:
```
~/Library/Mobile Documents/com~apple~CloudDocs/
```

Or use the dedicated **Files** app folder approach:
- On iPad: save screenshots to **Files → iCloud Drive → Screenshots**
- Mac path: `~/Library/Mobile Documents/com~apple~CloudDocs/Screenshots`

### Watch mode (leave running on your Mac)

```bash
python3 watch_and_import.py \
    ~/Library/Mobile\ Documents/com~apple~CloudDocs/Screenshots \
    --group "Fire Defence"
```

Every time you drop screenshots into the watched folder from your iPad
(they sync automatically via iCloud), the script:
1. Waits 3 seconds for the batch to finish syncing.
2. Stitches them vertically, trimming duplicate scroll regions.
3. Saves a PDF next to the originals.
4. Imports it into the specified DEVONthink group.

### One-off manual stitch

```bash
python3 stitch_to_devonthink.py \
    ~/Desktop/page1.png ~/Desktop/page2.png ~/Desktop/page3.png \
    --import \
    --group "Fire Defence"
```

---

## Choosing the right method

| Situation | Best method |
|-----------|-------------|
| Web page in Safari | Method 1 (Full Page screenshot) |
| PDF viewer, email, map, or any other app | Method 2 (Shortcut) |
| Need best quality / automatic trimming of overlap | Method 3 (Mac script) |
| No Mac available | Method 2 (Shortcut) |
