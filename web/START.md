# Start

Run this one command to start the server and open it fullscreen:

```bash
cd /home/caddy/PRODUCTION/web && python3 app.py & SRV=$!; sleep 2; firefox --kiosk http://127.0.0.1:5050; kill $SRV 2>/dev/null
```

To exit: press **Ctrl+Q** (kiosk mode) or **Ctrl+W** to close the window — the server will shut down automatically.

If you prefer normal fullscreen (exitable with **Esc** or **F11**), use:

```bash
cd /home/caddy/PRODUCTION/web && python3 app.py & SRV=$!; sleep 2; firefox --new-window http://127.0.0.1:5050 & sleep 3; xdotool key F11; wait; kill $SRV 2>/dev/null
```
