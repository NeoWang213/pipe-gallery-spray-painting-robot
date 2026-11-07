"""Create a clean upload folder and ZIP beside the working folder.

No GitHub authentication or network access is used. Existing destination files
are replaced only when they belong to this package; unexpected files abort.
"""
from pathlib import Path
import hashlib
import json
import shutil
import zipfile
from check_project import ROOT, upload_files, main as check


def main():
    check()
    files=list(upload_files())
    target=ROOT.with_name(ROOT.name+'-upload')
    expected={p.relative_to(ROOT).as_posix() for p in files}
    if target.exists():
        unknown=[p for p in target.rglob('*') if p.is_file() and p.relative_to(target).as_posix() not in expected]
        if unknown:
            raise RuntimeError(f'Unexpected files in upload directory: {unknown}')
    target.mkdir(exist_ok=True)
    for p in files:
        out=target/p.relative_to(ROOT)
        out.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,out)
        assert out.read_bytes()==p.read_bytes()
    archive=ROOT.with_name(ROOT.name+'-upload.zip')
    with zipfile.ZipFile(archive,'w',compression=zipfile.ZIP_DEFLATED) as z:
        for p in files:
            z.write(p,p.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(archive) as z:
        assert z.testzip() is None
        assert set(z.namelist())==expected
    manifest={p.name:{'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
              for p in (ROOT/'release-assets').iterdir() if p.is_file() and p.suffix.lower() in {'.mp4','.zip'}}
    (ROOT/'release-assets/SHA256SUMS.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('Upload folder:',target)
    print('Source ZIP:',archive)
    print('Original attachments:',ROOT/'release-assets')


if __name__=='__main__':
    main()
