"""Check the shareable tree for missing links, local paths, credentials and oversized files."""
import ast,json,re,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
EXCLUDED={'runs','build','.git','.venv','__pycache__'}
errors=[];files=[];total=0;largest=(0,'')
for p in ROOT.rglob('*'):
    rel=p.relative_to(ROOT)
    if any(part in EXCLUDED for part in rel.parts) or p.name=='.env':continue
    if p.is_symlink():
        if p.readlink().is_absolute() or not p.resolve().is_relative_to(ROOT) or not p.exists():errors.append('external/broken symlink: '+str(rel))
        continue
    if not p.is_file():continue
    files.append(p);size=p.stat().st_size;total+=size;largest=max(largest,(size,str(rel)))
    if size>=50*1024*1024:errors.append('file >=50MiB: '+str(rel))
    if p.suffix in ('.ulg','.so','.o','.bag','.db3','.mcap'):errors.append('generated binary/log: '+str(rel))
    if p.name.startswith('.env') and p.name!='.env.example':errors.append('local env: '+str(rel))
    if p.suffix in ('.py','.md','.json','.csv','.sh','.yml','.yaml','.txt','.sdf','.config','.repos','.cff') or p.name in ('LICENSE','.gitignore'):
        text=p.read_text()
        if re.search(r'/home/[A-Za-z0-9_.-]+/',text):errors.append('user-specific path: '+str(rel))
        if re.search(r'gh[pousr]_[A-Za-z0-9]{30,}|-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',text):errors.append('possible credential: '+str(rel))
        if p.suffix=='.py':ast.parse(text,filename=str(rel))
        if p.suffix=='.json':json.loads(text)
        if p.suffix=='.sh':
            result=subprocess.run(['bash','-n',str(p)],capture_output=True,text=True)
            if result.returncode:errors.append('shell syntax: '+str(rel))
        if p.suffix=='.md':
            for dest in re.findall(r'\]\(([^)]+)\)',text):
                dest=dest.split('#',1)[0]
                if not dest or '://' in dest or dest.startswith('mailto:'):continue
                if not (p.parent/dest).exists():errors.append('broken local document link: '+str(rel)+' -> '+dest)
required=['README.md','README.vi.md','LICENSE','THIRD_PARTY_NOTICES.md','.gitignore','.env.example','docs/INSTALL.md','docs/RELEASE_CHECKS.md']
for path in required:
    if not (ROOT/path).is_file():errors.append('missing required '+path)
print(json.dumps(dict(status='PASS' if not errors else 'FAIL',files=len(files),bytes=total,largest_file=dict(bytes=largest[0],path=largest[1]),errors=errors),indent=2))
raise SystemExit(bool(errors))
