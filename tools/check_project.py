"""Check archive hashes, Markdown links and upload limits without network access."""
from pathlib import Path
from urllib.parse import unquote
import re
import hashlib
import json

ROOT=Path(__file__).resolve().parents[1]
ALLOWED_DIRS={'software','config','archive','hardware','docs','tools','tests'}
ALLOWED_ROOT={
    'README.md','README.zh-CN.md','CHANGELOG.md','NOTICE.md','run.py','.gitignore','.gitattributes',
    'requirements-v1.txt','requirements-v2.txt','requirements-hardware-v2.txt',
}


def upload_files():
    for p in sorted(ROOT.rglob('*')):
        rel=p.relative_to(ROOT)
        if not p.is_file() or '__pycache__' in rel.parts:
            continue
        if len(rel.parts)==1 and rel.name in ALLOWED_ROOT:
            yield p
        elif len(rel.parts)>1 and rel.parts[0] in ALLOWED_DIRS:
            if rel.parts[0]=='config' and rel.name.startswith('local'):
                continue
            yield p


def main():
    files=list(upload_files())
    file_set={p.resolve() for p in files}
    for entry in json.loads((ROOT/'archive/source-manifest.json').read_text(encoding='utf-8')):
        p=ROOT/entry['archived']
        assert hashlib.sha256(p.read_bytes()).hexdigest()==entry['sha256'],str(p)
        original=ROOT/entry['original']
        if original.exists():
            assert original.read_bytes()==p.read_bytes(),str(original)
    for p in files:
        assert p.stat().st_size<100*1024**2,f'File too large for Git: {p}'
        assert p.suffix.lower() not in {'.mp4','.zip'},f'Attachment entered source set: {p}'
        if p.suffix=='.md':
            for target in re.findall(r'!?\[[^\]]*\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
                if target.startswith(('http://','https://','mailto:','#')):
                    continue
                target=unquote(target.split('#',1)[0])
                resolved=(p.parent/target).resolve()
                assert resolved.exists(),f'Broken link in {p.name}: {target}'
                if resolved.is_file():
                    assert resolved in file_set,f'Link outside source package: {target}'
    print(f'PASS: {len(files)} upload files, {sum(p.stat().st_size for p in files)/1024**2:.1f} MiB')
    print('PASS: original hashes, Markdown local links, Git file size limits, attachment separation')


if __name__=='__main__':
    main()
