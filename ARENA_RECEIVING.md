# Arena: receive and use the session

## Check the transfer

With the web app, ask for the single `*_RETURN.zip` and unzip it first. It contains a `.tar.gz` and matching `.tar.gz.sha256`. With the terminal workflow, ask for those two files directly. From the directory containing the received files, verify the archive before unpacking:

```bash
# macOS
shasum -a 256 -c RUN_ARCHIVE.tar.gz.sha256

# Linux
sha256sum -c RUN_ARCHIVE.tar.gz.sha256
```

Use the actual filename. It should print `OK`. For operator-created archives from this kit, unpack into a new empty receiving directory, preserving the run folder name:

```bash
mkdir received-session
 tar -xzf RUN_ARCHIVE.tar.gz -C received-session
```

Then use your local kit to check every returned payload:

```bash
/path/to/G1_SESSION_KIT/kit.sh verify /path/to/received-session/RUN_DIRECTORY
```

Open `report.html`. Keep the original archive unchanged. Work from a copy if adding analysis or labels later.

## Review in this order

1. Is `source` actually `physical_dds`, and are there nonzero state samples? Demo data is explicitly synthetic.
2. Was the stream present for the intended attempt? Inspect effective saved rate, long gaps, decoding errors, collector drops, error/crash status, and operator notes. Receiver counters count callbacks, not all source/network losses.
3. Can the exact policy be identified? Look for Arena submission/version, config/checkpoint evidence, launch log, robot identity, and operator. A supplied label is a claim until matched to actual artifacts.
4. Were command messages captured? If absent, state can still support motion/temperature diagnostics; it cannot establish commanded-versus-observed tracking.
5. Are all attempts accounted for, including failed launches and operator stops? Review `trial-start`, labels, corrections, unknowns, video, and interventions. A clean program exit is not task success.
6. Compare policies only on shared task definitions and conditions. For two unrelated policies, report separate diagnostic findings instead of ranking incomparable success fractions.

## Useful sim-to-real feedback

- Joint motion and measured velocity/estimated torque/temperature ranges during each attempt.
- Whether expected movement occurred and where a policy failed or was interrupted, supported by labeled trials and video.
- Data continuity and state/command availability, including policy exit errors.
- Requested command gains/targets, if observed, for engineering investigation alongside the state log.
- Policy-provided timing, detections, and internal failure reasons from optional hook/log attachments.
- Calibration, sensor recordings, checkpoint/config hashes and scene evidence needed to reproduce or refine simulation.

Build latency and tracking analyses only after validating message semantics and clock alignment. The report intentionally does not subtract an arbitrary nearest command from state and call it measured controller tracking error. External video/phone clocks are not automatically aligned.

## Relationship to existing eval packages

This kit captures evidence independently of the policy. It is not an automated importer into the existing `him-sports-eval` task scoring system. Formal table-tennis scoring still requires the frozen task schedule, independent incoming/landing reference measurements, trial mapping, and reviewed labels. Existing Astra timing/observation replay expects its declared calibrated observation format. Preserve these raw sessions first; create task-specific adapters and reference annotations after seeing the policies/data.

A useful feedback packet is the verified run archive plus a short interpretation of the task, key failure, relevant trial/time, supporting evidence, and what to change/test next. Distinguish measured diagnostics from hypotheses about why the policy failed.
