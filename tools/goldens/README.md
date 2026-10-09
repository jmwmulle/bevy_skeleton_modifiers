# Godot references

Source and executable: Godot 4.7.2-stable, commit
`ed1daf0bf001b61586d9930840f2f1394092c079`.
The official macOS universal release archive was verified against its published
SHA-512 list:
`38aa16e5bba2083941fc5b3e54be0089bd4cc35e32415f5b9fd9a8a6a7b9818255d44532ea8ef94b5aef56c4b407c2d634fa4f657e4ebe681ebbf59b7bac69ca`.

Set `GODOT` to that executable and run `tools/goldens/generate.sh`.
`dump.gd` constructs all 23 named scenarios without imported assets. It advances
Skeleton3D's native manual modifier pipeline and captures `modification_processed`
before the skeleton restores its input pose. An inactive zero-delta advance
registers the modifier before recording, letting deferred setup finish. The run
records 300 frames at fixed 60 fps; `--quit-after 301` allows the setup frame.
Every actual delta is retained, including its full JSON precision.

Fixtures are JSON compressed with deterministic gzip (`mtime=0`). Compression
changes no numeric values. The 23 fixtures total 2,398,643 bytes. A second complete
run produced identical SHA-256 hashes; `tests/goldens/manifest.json` records them.
`math.gd` emits shortest-arc quaternion references, including opposite vectors.

`upstream/` contains verbatim pinned source and its hash manifest. It is excluded
from the published crate, but retained in Git for independent inspection. The
MIT notice and human credits are retained in the distribution.

The temporary official executable and archive were removed after reproducibility verification, freeing 524,305,103 bytes (500.0 MiB). Regeneration requires obtaining that verified release again.

## Native Linux warm starts

The official Linux x86_64 Godot 4.7.2 archive is verified with SHA-512
`9aa00f7a605200940bce3027a567b782f49bd8e940dd06ae9e987bd65aee1b1467edd56ed84fcdcbdd44354bf613bdbb4e5d2913e925850368e150c59ed54c65`.
The native-reference workflow runs the unchanged FABRIK, CCD and Jacobian warm-start scenarios against this binary and compares every frame with the Rust core at the original tolerances. Its generated files are committed separately under tests/goldens/linux-x86_64 with a provenance/hash manifest, while the macOS fixtures remain unchanged. The workflow also verifies regenerated hashes against that manifest. The CI runner's disposable binary is not included in Git or the crate.
