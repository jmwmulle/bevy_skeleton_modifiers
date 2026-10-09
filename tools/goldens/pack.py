"""Compress reference output reproducibly without changing any numbers."""
import gzip, json, pathlib, sys
folder=pathlib.Path(sys.argv[1])
destination=pathlib.Path('tests/goldens')
destination.mkdir(parents=True,exist_ok=True)
for path in sorted(folder.glob('*.json')):
    if path.name == 'math.json': continue
    raw=path.read_bytes()
    data=json.loads(raw)
    assert len(data['frames'])==300, path
    destination.joinpath(path.name+'.gz').write_bytes(gzip.compress(raw,mtime=0))
print('Reference bytes:',sum(p.stat().st_size for p in destination.glob('*.gz')))
