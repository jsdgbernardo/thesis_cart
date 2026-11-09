#!/usr/bin/env python3
"""Reversibly add the independent TURN_OUT/PASS/RETURN baseline.

Supports either the original thesis trial scripts or those already patched for
stop-only local_avoid; does not overwrite historical trial folders.
"""
from pathlib import Path
import shutil
import sys

BASE = Path.home() / 'thesis_cart'
runner = BASE / 'run_baseline_trial.sh'
analyzer = BASE / 'analyze_baselines.py'
new_node = Path(__file__).resolve().parent / 'reactive_avoid_cart.py'
for p in (runner, analyzer, new_node):
    if not p.is_file():
        sys.exit(f'ERROR: Missing file: {p}')

r0, a0 = runner.read_text(), analyzer.read_text()
if 'reactive_avoid' in r0 or "'_reactive_avoid_'" in a0:
    sys.exit('ERROR: Partially or already patched. Review current scripts and backups before continuing.')
r, a = r0, a0

# 1. Register the new method.
old = 'if [[ "$METHOD" != "no_comm" && "$METHOD" != "fcfs"'
if r.count(old) != 1:
    sys.exit('Runner method-validation pattern not found; unchanged.')
r = r.replace(old, old + ' && "$METHOD" != "reactive_avoid"', 1)
r = r.replace('no_comm|local_avoid|fcfs', 'no_comm|local_avoid|reactive_avoid|fcfs')
r = r.replace('no_comm|fcfs', 'no_comm|reactive_avoid|fcfs')

# 2. Add controller start branch just before existing no_comm/local_avoid.
start = '''if [[ "$METHOD" == reactive_avoid ]]; then
  for ((i=1; i<=COUNT; i++)); do
    /usr/bin/python3 "$BASE/reactive_avoid_cart.py" \\
      --cart-id "$i" --carts "$COUNT" --speed 0.5 \\
      --stop-radius 1.2 --protected-separation 0.616 \\
      --prediction-horizon 3.0 --sensor-range 3.3 \\
      --aisle-width "$WIDTH" --world-half-width "$HALF" \\
      > "$RUN/cart${i}.log" 2>&1 &
    PIDS+=("$!")
  done
elif '''
if 'if [[ "$METHOD" == local_avoid ]]; then\n  for ((i=1; i<=COUNT; i++)); do' in r:
    needle = 'if [[ "$METHOD" == local_avoid ]]; then\n  for ((i=1; i<=COUNT; i++)); do'
    replacement = start + needle[len('if '):]
elif 'if [[ "$METHOD" == no_comm ]]; then\n  /usr/bin/python3' in r:
    needle = 'if [[ "$METHOD" == no_comm ]]; then\n  /usr/bin/python3'
    replacement = start + needle[len('if '):]
else:
    sys.exit('Runner controller-launch block not recognized; unchanged.')
r = r.replace(needle, replacement, 1)

# 3. Add metadata.
needle = 'PIDS=()\nBAG_PID=\'\''
if r.count(needle) != 1:
    sys.exit('Runner PIDS marker missing; unchanged.')
r = r.replace(needle, '''if [[ "$METHOD" == reactive_avoid ]]; then
  printf 'intercart_messages=disabled\\nlocal_avoidance=turn_out_pass_return\\nsensor_proxy=Gazebo_model_states\\nsensing_range_m=3.3\\nsafety_radius_m=1.2\\nprotected_separation_m=0.616\\n' >> "$RUN/setup.txt"
fi

''' + needle, 1)

# 4. Allow ENTER during tests or an 85-second upper observation window.
old = "read -r -p 'Press ENTER to stop this trial: ' _"
if r.count(old) == 1:
    r = r.replace(old, '''if [[ "$METHOD" == reactive_avoid ]]; then
  echo 'Observe until all carts exit, overlap, or stall; press ENTER to stop.'
  read -r -t 85 -p 'Press ENTER to finish reactive trial: ' _ || true
else
  ''' + old + '''
fi''', 1)
else:
    needle = '''if [[ "$METHOD" == local_avoid ]]; then
  echo 'Observe for completion or a persistent deadlock.'''
    if r.count(needle) != 1:
        sys.exit('Runner end-of-trial prompt pattern missing; unchanged.')
    r = r.replace(needle, '''if [[ "$METHOD" == reactive_avoid ]]; then
  echo 'Observe until all carts exit, overlap, or stall; press ENTER to stop.'
  read -r -t 85 -p 'Press ENTER to finish reactive trial: ' _ || true
elif [[ "$METHOD" == local_avoid ]]; then
  echo 'Observe for completion or a persistent deadlock.''', 1)

# 5. Register new analysis grouping, preserving old raw bags.
needle = "method = 'local_avoid' if '_local_avoid_' in run.name else \\\n             'no_comm' if '_no_comm_' in run.name else \\"
if needle in a:
    a = a.replace(needle, "method = 'reactive_avoid' if '_reactive_avoid_' in run.name else \\\n             " + needle.split('method = ', 1)[1], 1)
else:
    needle = "method = 'no_comm' if '_no_comm_' in run.name else \\"
    if a.count(needle) != 1:
        sys.exit('Analysis method classifier pattern missing; unchanged.')
    a = a.replace(needle, "method = 'reactive_avoid' if '_reactive_avoid_' in run.name else \\\n             " + needle.split('method = ', 1)[1], 1)

if "for method in ('local_avoid', 'fcfs', 'proposed'):" in a:
    a = a.replace("for method in ('local_avoid', 'fcfs', 'proposed'):",
                  "for method in ('reactive_avoid', 'fcfs', 'proposed'):", 1)
elif "for method in ('no_comm', 'fcfs', 'proposed'):" in a:
    a = a.replace("for method in ('no_comm', 'fcfs', 'proposed'):",
                  "for method in ('reactive_avoid', 'fcfs', 'proposed'):", 1)
else:
    sys.exit('Analysis summary grouping pattern missing; unchanged.')

# Validate syntax before writing the user's files.
import ast
try:
    ast.parse(a)
except SyntaxError as e:
    sys.exit(f'Generated analyzer syntax error; no changes: {e}')
for path in (runner, analyzer):
    backup = path.with_name(path.name + '.pre_reactive_avoid.bak')
    if backup.exists():
        sys.exit(f'Backup already exists; avoid accidental overwrite: {backup}')
for path in (runner, analyzer):
    shutil.copy2(path, path.with_name(path.name + '.pre_reactive_avoid.bak'))
shutil.copy2(new_node, BASE / 'reactive_avoid_cart.py')
runner.write_text(r)
analyzer.write_text(a)
print('Installed method: reactive_avoid (independent local turning/passing/recovery).')
print('Old no_comm, local_avoid, fcfs, proposed data unchanged.')
print('Backups are *.pre_reactive_avoid.bak')
