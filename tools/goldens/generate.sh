#!/usr/bin/env bash
set -euo pipefail
: "${GODOT:?Set GODOT to the verified Godot 4.7.2 executable}"
cd "$(dirname "$0")/../.."
temporary=$(mktemp -d)
trap 'rm -rf "$temporary"' EXIT
for scenario in spring_stiff spring_loose spring_gravity spring_sphere spring_capsule spring_inside_sphere spring_plane spring_hinge spring_center_bone spring_external two_sweep two_virtual fabrik_free fabrik_limits fabrik_warm ccd_free ccd_limits ccd_warm jacobian_free jacobian_limits jacobian_warm spline_plain spline_tilt; do
 "$GODOT" --headless --fixed-fps 60 --quit-after 301 --path tools/goldens/godot_project --script dump.gd -- "$scenario" "$temporary/$scenario.json"
done
"$GODOT" --headless --path tools/goldens/godot_project --script math.gd -- "$temporary/math.json"
cp "$temporary/math.json" tests/goldens/math.json
python3 tools/goldens/pack.py "$temporary"
