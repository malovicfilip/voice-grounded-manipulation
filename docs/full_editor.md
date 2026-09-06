# Full Isaac Sim editor on Brev

Use NVIDIA's Isaac Sim 6.0.1 Full Streaming App for the editor on the headless
Brev VM. This is the actual editor UI, transported through WebRTC, not a
recording. A VNC desktop is not required. NVIDIA cautions that windowed
container GUI mode requires a real display:

- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_faq.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/install_container.html

Before switching, stop the microphone console and shut down the integrated
session through `task_cli --session SESSION_ID --operation shutdown`. Verify
the simulator and its MoveIt processes have exited. Do not run both modes.
Keep the existing Windows/Brev shutdown guard armed, Windows online, and the
existing streaming ports restricted to your client. This launcher does not
configure firewall rules, start cloud compute, or install dependencies.

On the existing VM:

```bash
cd /home/ubuntu/workspace
ACCEPT_EULA=Y bash isaac_sim/scripts/run_full_editor.sh SERVER_IPV4
```

The command starts the cached NVIDIA image with the full editor UI. Source is
mounted read-only, output is writable, and ROS domain 42 separates the editor
from the normal domain-0 manipulation runtime. It does not launch the LLM,
microphone, task coordinator, MoveIt, or RGB-D capture request loop.

Open the existing viewer at `http://SERVER_IPV4:8210/` in a single browser tab.
In the editor choose **File > Open** and select:

```text
/workspace/isaac_sim/_output/safety-live-v2/integrated_scene.usd
```

This saved scene contains the Panda, table, objects and camera. It is a saved
stage, not the previous live session's in-memory state. Inspect the Stage and
Property panels and navigate the viewport. Leave playback paused initially.
Use **File > Save As** to a new filename in `/workspace/isaac_sim/_output/`;
do not overwrite the original capture. Output is ignored by Git: important
edits must be reviewed and exported into project source or scene assets, then
committed and pushed separately before discarding the VM.

To stop just the editor, run `docker stop --time 20 vgm-full-editor` on Brev.
Stopping a container does not stop VM compute billing. When finished with
the VM, run `brev stop vgm-isaac-dev` from WSL and verify with `brev ls`.
Return to voice manipulation by launching a fresh integrated session and
binding the local console to its new session ID; the editor is not a
replacement for the project's validated execution pipeline.
