# Contributing

Start with the offline checks in README.md. Keep new runtime output in ignored `runs/`. Add meaningful regression checks for changes to geometry, service credit, time mapping, formation behavior or trajectory profiles.

When proposing algorithm improvements, state which parameters, layouts and controls are held fixed. Keep sensor-service coverage separate from received bytes, energy proxy separate from measured battery energy, and ARM deadlines separate from MISSION time. Preserve unsuccessful/truncated cases in your own evidence archive; do not silently replace historical results in `results/` with a more favorable run.

Changes to source version pins, namespace/message compatibility, safety margins or phase behavior need an explanation and appropriate offline/SITL evidence. A passing offline workflow alone does not certify a new flight behavior. Do not commit local credentials, raw ULogs, build artifacts or reports containing personal academic information.
