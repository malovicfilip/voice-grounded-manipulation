# Browser command console

The browser command console is now the [unified demo dashboard](demo_dashboard.md).
It supports both the original exact-task confirmation flow and the new bounded
closed-loop agent flow from the same localhost-only interface.

## Recommended launch

```bash
./demo replay
```

or, for a running remote Isaac session:

```bash
./demo run \
  --mode remote \
  --control agent \
  --session YOUR_SESSION_ID \
  --host vgm-isaac-dev \
  --viewer-url http://YOUR_SERVER_IPV4:8210/
```

Open `http://localhost:8766/` if needed.

Use `--control task` for the original fixed planned-task workflow. See
[Closed-loop LLM control](agent_control.md) for the authority model and
[Demo dashboard](demo_dashboard.md) for replay/local/remote modes, diagnostics,
viewer embedding, microphone behavior, and security details.

STOP remains outside the LLM path. Closing the browser tab does not stop robot motion
or cloud billing; maintain the independent terminal STOP path during a live run.
