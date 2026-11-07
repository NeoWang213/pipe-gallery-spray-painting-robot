"""Generate editable SVG and PNG control/system diagrams (Pillow required)."""
from pathlib import Path
from html import escape
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'docs/images'
FONT = 'C:/Windows/Fonts/msyh.ttc'


class Diagram:
    def __init__(self, title, subtitle):
        self.image = Image.new('RGB', (1440, 900), '#f5f8fc')
        self.draw = ImageDraw.Draw(self.image)
        self.svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1440" height="900" viewBox="0 0 1440 900">',
                    '<rect width="1440" height="900" fill="#f5f8fc"/>']
        self.text(45,30,title,36,'#142d44')
        self.text(46,90,subtitle,20,'#51667a')

    def text(self,x,y,value,size=23,color='#142d44'):
        self.draw.text((x,y),value,font=ImageFont.truetype(FONT,size),fill=color)
        self.svg.append(f'<text x="{x}" y="{y+size}" fill="{color}" font-size="{size}" font-family="Microsoft YaHei,Noto Sans CJK SC,sans-serif">{escape(value)}</text>')

    def box(self,xy,lines,color='#e4eff8'):
        x0,y0,x1,y1=xy
        self.draw.rounded_rectangle(xy,radius=12,fill=color,outline='#b6ccdf',width=2)
        self.svg.append(f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" rx="12" fill="{color}" stroke="#b6ccdf" stroke-width="2"/>')
        for i,line in enumerate(lines):
            self.text(x0+18,y0+18+i*34,line,23 if i==0 else 19)

    def arrow(self,points,color='#2276ad'):
        self.draw.line(points,fill=color,width=4)
        self.svg.append('<polyline points="'+' '.join(f'{x},{y}' for x,y in points)+f'" fill="none" stroke="{color}" stroke-width="4"/>')
        x,y=points[-1]; px,py=points[-2]
        if x==px:
            a=[(x,y),(x-8,y-13 if y>py else y+13),(x+8,y-13 if y>py else y+13)]
        else:
            a=[(x,y),(x-13 if x>px else x+13,y-8),(x-13 if x>px else x+13,y+8)]
        self.draw.polygon(a,fill=color)
        self.svg.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in a)+f'" fill="{color}"/>')

    def save(self,name):
        OUT.mkdir(parents=True,exist_ok=True)
        self.image.save(OUT/f'{name}.png')
        (OUT/f'{name}.svg').write_text('\n'.join(self.svg+['</svg>']),encoding='utf-8')


def build():
    d=Diagram('喷枪电控原理示意','根据项目描述重绘 · 信号与功率通路示意 · 非原始接线图')
    d.box((45,190,285,305),['手机端蓝牙控制','原项目手机端工具'])
    d.box((365,190,625,305),['蓝牙模块','型号与接口未保留'])
    d.box((365,395,625,525),['STM32F103RCT6','控制信号输出','小电流控制侧'])
    d.box((45,395,285,525),['OLED 显示模块','显示状态','型号与接口未保留'])
    d.box((745,395,1005,525),['继电器控制模块','控制侧 / 触点侧','模块内部驱动未展开'])
    d.box((745,190,1005,305),['电磁阀供电','实际电压、额定电流','待依据实物确认'],'#fcebdc')
    d.box((1110,395,1390,525),['电磁阀','由继电器触点控制','功率回路通断'],'#fcebdc')
    d.box((1110,640,1390,755),['喷枪控制气路','电磁阀控制启闭','雾化与供漆另见气路图'],'#dff1eb')
    d.arrow([(285,248),(365,248)])
    d.arrow([(495,305),(495,395)])
    d.arrow([(365,460),(285,460)])
    d.arrow([(625,460),(745,460)])
    d.text(645,418,'信号',18)
    d.arrow([(875,305),(875,395)],'#bc6a27')
    d.arrow([(1005,495),(1110,495)],'#bc6a27')
    d.text(1018,552,'触点控制的功率通路',18,'#9a5420')
    d.arrow([(1250,525),(1250,640)],'#26826c')
    d.text(45,640,'控制说明',25)
    d.text(45,685,'蓝牙与 OLED 为后期扩展；上位机软件负责机械臂运动。',21)
    d.text(45,725,'喷枪通过独立遥控开关；现有 Python 文件未集成喷枪自动联动。',21)
    d.text(45,810,'未推定引脚、供电电压、保护元件或共地方式；实际接线须依据器件资料与实物确认。',20,'#51667a')
    d.save('electrical-control')

    d=Diagram('机器人系统与作业流程','人工遥控到位 → 视觉选区 → 轨迹规划与预览 → 机械臂执行；底盘作业时固定')
    d.box((45,190,350,320),['操作人员 / 上位机','v1.0：整机串口控制','v2.0：视觉喷涂专项软件'])
    d.box((490,190,875,320),['视觉与轨迹规划','RealSense 深度图像','选区、投影、往返扫描、预览'])
    d.box((1010,190,1390,320),['UFACTORY xArm6','网口 / xArm SDK','执行预定喷涂轨迹'])
    d.arrow([(350,250),(490,250)])
    d.arrow([(875,250),(1010,250)])
    d.box((45,455,350,585),['麦克纳姆轮全向底盘','遥控调整作业位置','喷涂期间保持固定'])
    d.box((490,455,875,585),['独立喷漆系统','STM32、继电器、电磁阀','蓝牙开关与 OLED 状态显示'])
    d.box((1010,455,1390,585),['末端喷枪 / 相机装配','独立设计的连接与支架','气动回路、供漆与喷枪控制'])
    d.arrow([(200,320),(200,455)])
    d.arrow([(875,520),(1010,520)],'#26826c')
    d.arrow([(1200,320),(1200,455)],'#26826c')
    d.text(45,690,'已完成验证',27)
    d.text(45,740,'平面试件上的定位、轨迹执行与实际喷漆；独立遥控喷枪开关。',24)
    d.text(45,800,'目标应用：管廊内管道外壁维护。管道曲面姿态跟随与定量涂层质量尚无已提供验证。',21,'#51667a')
    d.save('system-architecture')


if __name__=='__main__':
    build()
