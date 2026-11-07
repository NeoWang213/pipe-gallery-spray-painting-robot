"""Rebuild portfolio images and previews from local original materials.

Requires Pillow, OpenCV and numpy; videos stay local in release-assets/.
"""
from pathlib import Path
import json
import cv2
from PIL import Image, ImageOps, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
IMAGES = ROOT / 'docs/images'
PREVIEWS = ROOT / 'docs/previews'
TMP = ROOT / 'tmp'
FONT = Path('C:/Windows/Fonts/msyh.ttc')


def font(size):
    return ImageFont.truetype(str(FONT), size)


def fit(path, size):
    with Image.open(path) as src:
        image = ImageOps.exif_transpose(src).convert('RGB')
        return ImageOps.contain(image, size, Image.Resampling.LANCZOS)


def build():
    for folder in (IMAGES, PREVIEWS, TMP):
        folder.mkdir(parents=True, exist_ok=True)
    files = {
        'robot.jpg': '喷漆机器人整体照片.jpg',
        'chassis.jpg': '机器人底盘.jpg',
        'end-effector.png': '机械臂末端喷枪3D模型.png',
        'pneumatic-original.png': '喷漆气动通路原理图.png',
        'controller.jpg': '喷枪电控平台.jpg',
        'pneumatic.jpg': '气动通路.jpg',
    }
    for dest, src in files.items():
        image = fit(ROOT / src, (1600, 1300))
        image.save(IMAGES / dest, **({'quality': 88} if dest.endswith('.jpg') else {}))

    board = Image.new('RGB', (1440, 1000), '#101c2a')
    draw = ImageDraw.Draw(board)
    draw.text((46, 30), '小型管廊喷漆机器人', font=font(46), fill='#f2f7fc')
    draw.text((48, 96), '移动平台 · 六自由度机械臂 · 独立喷漆系统 · 视觉轨迹规划', font=font(23), fill='#79d4db')
    cards = [('robot.jpg', '整机集成', (40, 160, 890, 950)),
             ('end-effector.png', '末端喷枪与相机装配模型', (920, 160, 1400, 525)),
             ('controller.jpg', '独立喷枪电控平台', (920, 555, 1400, 950))]
    for file, label, (x0, y0, x1, y1) in cards:
        draw.rounded_rectangle((x0, y0, x1, y1), radius=16, fill='#203043')
        draw.text((x0+20, y0+15), label, font=font(24), fill='white')
        im = fit(IMAGES/file, (x1-x0-26, y1-y0-80))
        board.paste(im, (x0+(x1-x0-im.width)//2, y0+65+(y1-y0-80-im.height)//2))
    board.save(IMAGES/'project-overview.jpg', quality=90)

    metadata = []
    contact = Image.new('RGB', (1200, 5*300), '#edf2f7')
    draw = ImageDraw.Draw(contact)
    previews = {
        '完整客户端演示视频.mp4': 'client',
        '定位演示视频.mp4': 'positioning',
        '喷漆定位验证视频.mp4': 'painting',
        '蓝牙控制喷枪开关演示视频.mp4': 'bluetooth',
    }
    for row, (name, stem) in enumerate(previews.items()):
        path = ROOT/'release-assets'/name
        cap = cv2.VideoCapture(str(path))
        fps = cap.get(cv2.CAP_PROP_FPS)
        total = cap.get(cv2.CAP_PROP_FRAME_COUNT)
        duration = total/fps if fps else 0
        metadata.append({'file': name, 'duration_seconds': round(duration, 2),
                         'width': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                         'height': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                         'bytes': path.stat().st_size})
        frames=[]
        for i in range(18):
            seconds=duration*(0.08+0.84*i/17)
            cap.set(cv2.CAP_PROP_POS_MSEC, seconds*1000)
            ok, frame=cap.read()
            if not ok:
                continue
            image=Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            image.thumbnail((640, 400), Image.Resampling.LANCZOS)
            canvas=Image.new('RGB',(640,440),'#101c2a')
            canvas.paste(image,((640-image.width)//2,(400-image.height)//2))
            ImageDraw.Draw(canvas).text((12,408),f'原始视频抽帧预览 | {seconds:.1f}s / {duration:.1f}s',font=font(16),fill='white')
            frames.append(canvas)
            if i in (2,8,14):
                thumb=ImageOps.contain(canvas,(390,260))
                contact.paste(thumb,((0 if i==2 else 400 if i==8 else 800),row*300+32))
        if frames:
            frames[0].save(PREVIEWS/f'{stem}.gif',save_all=True,append_images=frames[1:],duration=360,loop=0,optimize=True)
            frames[8 if len(frames)>8 else 0].save(IMAGES/f'{stem}-still.jpg',quality=90)
        draw.text((8,row*300+4),name,font=font(18),fill='#172c40')
        cap.release()
    (ROOT/'docs/media-metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    contact.crop((0,0,1200,1200)).save(TMP/'video-contact.jpg')
    print(json.dumps(metadata,ensure_ascii=False,indent=2))


if __name__=='__main__':
    build()
