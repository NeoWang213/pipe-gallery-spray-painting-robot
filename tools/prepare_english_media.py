"""English presentation graphics and preview captions, preserving source footage.

Run after prepare_media.py. Requires Pillow; editable diagrams reuse draw_diagrams.
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont, ImageSequence
from draw_diagrams import Diagram
from prepare_media import fit

ROOT=Path(__file__).resolve().parents[1]
IMAGES=ROOT/'docs/images'
PREVIEWS=ROOT/'docs/previews'
FONT='C:/Windows/Fonts/segoeui.ttf'
BOLD='C:/Windows/Fonts/segoeuib.ttf'


def font(size,bold=False):
    return ImageFont.truetype(BOLD if bold else FONT,size)


class EnglishDiagram(Diagram):
    def text(self,x,y,value,size=23,color='#142d44'):
        from html import escape
        self.draw.text((x,y),value,font=font(size),fill=color)
        self.svg.append(f'<text x="{x}" y="{y+size}" fill="{color}" font-size="{size}" font-family="Segoe UI,Arial,sans-serif">{escape(value)}</text>')

    def box(self,xy,lines,color='#e4eff8'):
        x0,y0,x1,y1=xy
        self.draw.rounded_rectangle(xy,radius=12,fill=color,outline='#b6ccdf',width=2)
        self.svg.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="12" fill="{color}" stroke="#b6ccdf" stroke-width="2"/>')
        for i,line in enumerate(lines):
            size=23 if i==0 else 19
            while self.draw.textlength(line,font=font(size))>x1-x0-36:
                size-=1
            assert size>=15,f'Text too long for box: {line}'
            self.text(x0+18,y0+18+i*34,line,size)


def overview():
    board=Image.new('RGB',(1440,1000),'#101c2a')
    draw=ImageDraw.Draw(board)
    draw.text((46,30),'Pipe-Gallery Spray-Painting Robot',font=font(46,True),fill='#f2f7fc')
    draw.text((48,96),'Mobile base  |  Six-axis arm  |  Custom spray system  |  Visual path planning',font=font(25),fill='#79d4db')
    cards=[('robot.jpg','Robot integration',(40,160,890,950)),
           ('end-effector.png','Spray gun + camera assembly',(920,160,1400,525)),
           ('controller.jpg','Custom spray control platform',(920,555,1400,950))]
    for file,label,(x0,y0,x1,y1) in cards:
        draw.rounded_rectangle((x0,y0,x1,y1),radius=16,fill='#203043')
        draw.text((x0+20,y0+15),label,font=font(24,True),fill='white')
        im=fit(IMAGES/file,(x1-x0-26,y1-y0-80))
        board.paste(im,(x0+(x1-x0-im.width)//2,y0+65+(y1-y0-80-im.height)//2))
    board.save(IMAGES/'project-overview-en.jpg',quality=90)


def architecture():
    d=EnglishDiagram('Robot System and Painting Workflow',
        'Position the base > Select a region > Plan and preview > Execute the arm path; keep the base stationary')
    d.box((45,190,350,320),['Operator / desktop app','v1.0: serial robot control','v2.0: visual painting app'])
    d.box((490,190,875,320),['Vision and path planning','RealSense depth data','Selection, projection, raster path'])
    d.box((1010,190,1390,320),['UFACTORY xArm6','Ethernet / xArm SDK','Execute the planned arm path'])
    d.arrow([(350,250),(490,250)])
    d.arrow([(875,250),(1010,250)])
    d.box((45,455,350,585),['Mecanum-wheel base','Remote positioning','Stationary during painting'])
    d.box((490,455,875,585),['Custom spray system','STM32, relay, solenoid valve','Bluetooth switching + OLED'])
    d.box((1010,455,1390,585),['Spray gun / camera assembly','Custom mounts and brackets','Pneumatic circuit and paint supply'])
    d.arrow([(200,320),(200,455)])
    d.arrow([(875,520),(1010,520)],'#26826c')
    d.arrow([(1200,320),(1200,455)],'#26826c')
    d.text(45,690,'Physical demonstrations',27)
    d.text(45,740,'Positioning, path execution, and spray painting on planar test pieces.',24)
    d.text(45,782,'The spray gun is switched through a separate remote control.',22)
    d.text(45,837,'Target: pipe exterior maintenance. Curved-pipe following and quantitative coating quality remain unverified.',20,'#51667a')
    d.save('system-architecture-en')


def electrical():
    d=EnglishDiagram('Spray-Gun Electrical Control',
        'Reconstructed from the project description | Functional signal and power paths | Not a wiring diagram')
    d.box((45,190,285,305),['Mobile control','Original Bluetooth app'])
    d.box((365,190,625,305),['Bluetooth module','Model and interface','not retained'])
    d.box((365,395,625,525),['STM32F103RCT6','Control signal output','Low-current control'])
    d.box((45,395,285,525),['OLED display','Status indication','Model not retained'])
    d.box((745,395,1005,525),['Relay module','Control / contact sides','Driver details not shown'])
    d.box((745,190,1005,305),['Valve power supply','Voltage and rated current','require confirmation'],'#fcebdc')
    d.box((1110,395,1390,525),['Solenoid valve','Switched by relay contacts','Power circuit on / off'],'#fcebdc')
    d.box((1110,640,1390,755),['Spray control air','Valve opens / closes','See pneumatic diagram'],'#dff1eb')
    d.arrow([(285,248),(365,248)])
    d.arrow([(495,305),(495,395)])
    d.arrow([(365,460),(285,460)])
    d.arrow([(625,460),(745,460)])
    d.text(645,418,'Signal',18)
    d.arrow([(875,305),(875,395)],'#bc6a27')
    d.arrow([(1005,495),(1110,495)],'#bc6a27')
    d.text(1018,552,'Relay-switched power',18,'#9a5420')
    d.arrow([(1250,525),(1250,640)],'#26826c')
    d.text(45,640,'Control notes',25)
    d.text(45,685,'Bluetooth and OLED were added later; the desktop app controls arm motion.',21)
    d.text(45,725,'Spray switching uses a separate remote control, without automatic desktop linkage.',21)
    d.text(45,810,'Pin assignments, voltage, protection components, and grounding details are not inferred.',20,'#51667a')
    d.text(45,845,'Actual wiring must follow the hardware and component documentation.',20,'#51667a')
    d.save('electrical-control-en')


def pneumatic():
    d=EnglishDiagram('Spray System Pneumatic Circuit',
        'English rendering of the original project diagram | Air, control, and paint paths')
    d.box((45,180,335,300),['Air compressor','Compressed air source'])
    d.box((440,180,685,300),['Tee fitting','Two air branches'])
    d.box((800,180,1100,300),['Pressure regulator','Atomization air branch'])
    d.box((800,420,1100,540),['Solenoid valve','Spray control air branch'])
    d.box((1190,300,1395,465),['Spray gun','Atomization air','Control air','Paint inlet'])
    d.box((440,650,730,770),['Paint pressure tank','Paint supply'])
    d.box((800,650,1100,770),['Control system','Electrical valve command'])
    d.arrow([(335,240),(440,240)])
    d.arrow([(685,240),(800,240)])
    d.arrow([(560,300),(560,480),(800,480)])
    d.arrow([(1100,240),(1290,240),(1290,300)])
    d.text(1123,195,'Atomization air',19)
    d.arrow([(1100,480),(1150,480),(1150,395),(1190,395)])
    d.text(1115,511,'Control air',19)
    d.arrow([(950,650),(950,540)],'#bc6a27')
    d.text(972,580,'Electrical control',19,'#9a5420')
    d.arrow([(585,770),(585,790),(1340,790),(1340,465)],'#26826c')
    d.text(1125,799,'Paint supply',19,'#26826c')
    d.text(45,835,'Connections follow the original schematic; component ratings and operating settings were not provided.',20,'#51667a')
    d.save('pneumatic-en')


def previews():
    for stem in ('client','painting','positioning','bluetooth'):
        frames=[]
        with Image.open(PREVIEWS/f'{stem}.gif') as gif:
            duration=gif.info.get('duration',360)
            for frame in ImageSequence.Iterator(gif):
                im=frame.convert('RGB')
                draw=ImageDraw.Draw(im)
                # Translate the authored footer; original recorded interfaces stay intact.
                draw.rectangle((0,400,639,439),fill='#101c2a')
                # Same uniformly sampled source times as prepare_media.py.
                metadata=__import__('json').loads((ROOT/'docs/media-metadata.json').read_text(encoding='utf-8'))
                index={'client':0,'positioning':1,'painting':2,'bluetooth':3}[stem]
                total=metadata[index]['duration_seconds']
                seconds=total*(0.08+0.84*len(frames)/17)
                draw.text((12,408),f'Sampled original footage | {seconds:.1f}s / {total:.1f}s',font=font(17),fill='white')
                frames.append(im)
        frames[0].save(PREVIEWS/f'{stem}-en.gif',save_all=True,append_images=frames[1:],duration=duration,loop=0,optimize=True)
    print('English overview, architecture, electrical/pneumatic diagrams, and four captioned GIFs generated.')


if __name__=='__main__':
    overview()
    architecture()
    electrical()
    pneumatic()
    previews()
