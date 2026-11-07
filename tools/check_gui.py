"""Offline GUI integration checks; never opens physical devices.

Run each version in a separate process: python tools/check_gui.py v1|v2
"""
from pathlib import Path
import sys
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from software import settings
settings.configure(offline=True)
import importlib
import time
from types import SimpleNamespace

version=sys.argv[1]
m=importlib.import_module(f'software.{version}')
app=m.QApplication([])
from software.desktop import setup_fonts
setup_fonts(app,m.QFontDatabase,m.QFont)
window=m.MainWindow()
window.show()
app.processEvents()
assert not window.controller.is_connected if version=='v1' else not window.arm.connected

if version=='v1':
    window.update_joint(0,10)
    window.move_chassis('forward')
    window.update_status()
    assert '未接入' in window.distance_label.text()
    assert not window.cam_threads
    assert not window.controller.connect('NOT_A_REAL_PORT')
    # Exercise the original packet encoder without opening any port.
    packets=[]
    controller=m.SprayController()
    controller.is_connected=True
    controller.ser=SimpleNamespace(write=lambda p: packets.append(p) or len(p))
    controller.send_command(movement=(0.5,0,0))
    assert packets[0][:3]==bytes([0xAA,2,12])
    assert packets[0][-2]==sum(packets[0][:-2]) & 0xFF
    assert packets[0][-1]==0x55
    controller.ser=SimpleNamespace(write=lambda p:0)
    try:
        controller.send_command(movement=(0.5,0,0))
    except m.serial.SerialException:
        pass
    else:
        raise AssertionError('Short serial write was reported as success')
    assert not window.calibration_loaded
    window.save_calibration()
    assert not (settings.RUNTIME/'camera_calibration.yaml').exists()
    app.processEvents()
    window.grab().save(str(ROOT/'docs/images/curated-v1-offline.png'))
    print('PASS v1: offline startup, disconnected controls, non-simulated distance, serial encoding/failure, missing calibration')
else:
    from PySide6.QtTest import QTest
    from PySide6.QtCore import QPoint, Qt, QEventLoop, QTimer
    from software.execution import Outcome
    QTest.mouseClick(window.video_label,Qt.LeftButton,pos=QPoint(200,120))
    QTest.mouseClick(window.video_label,Qt.LeftButton,pos=QPoint(450,300))
    assert window.cam.rect_confirmed
    points,xyz=window.generate_trajectory()
    assert len(points)>2 and xyz is None
    window.current_trajectory=points
    window.execute_paint_trajectory()
    assert window.worker is None
    window.use_3d=True
    assert window.generate_trajectory()==(None,None)
    assert window.current_trajectory==[]
    # A valid frame projects all points; one missing point rejects the whole path.
    window.cam.rs_camera=SimpleNamespace(camera_info={'fx':500,'fy':500,'cx':320,'cy':240},
        get_depth_at_pixel=lambda x,y,frame:frame(x,y))
    window.cam.latest_depth=lambda x,y:500
    window.cam.latest_time=time.monotonic()
    _,xyz=window.generate_trajectory()
    assert xyz and '0.' in window.area_label.text()
    window.cam.latest_depth=lambda x,y:0 if x==200 else 500
    assert window.generate_trajectory()==(None,None)
    window.cam.latest_depth=lambda x,y:500
    window.cam.latest_time=time.monotonic()-10
    assert window.generate_trajectory()==(None,None)
    # The execution worker yields the GUI thread; cancellation wins over success.
    arm=m.ArmThread()
    calls=[]
    def move(*p,**kwargs):
        calls.append(p)
        time.sleep(0.15)
        return 0
    arm.arm=SimpleNamespace(connected=True,set_position=move,emergency_stop=lambda:0)
    arm.connected=True
    arm.executor.reset()
    worker=m.TrajectoryWorker(arm,[],[(300,0,200),(310,0,200)])
    loop=QEventLoop()
    responsive=[]
    QTimer.singleShot(40,lambda:(responsive.append(True),arm.emergency_stop()))
    worker.finished.connect(loop.quit)
    QTimer.singleShot(3000,loop.quit)
    worker.start()
    loop.exec()
    assert worker.wait(1000)
    assert responsive and worker.result==Outcome.STOPPED and len(calls)==1
    # Restore the actual offline preview before capturing a labelled screenshot.
    window.cam.rs_camera=None
    window.use_3d=False
    window.cam.latest_depth=None
    window.cam.rect_x0,window.cam.rect_y0=200,120
    window.cam.rect_x1,window.cam.rect_y1=450,300
    window.cam.rect_confirmed=True
    points,_=window.generate_trajectory()
    window.cam.set_trajectory_and_show(points)
    QTest.qWait(400)
    app.processEvents()
    window.grab().save(str(ROOT/'docs/images/curated-v2-offline.png'))
    dialog=m.TrajectoryPreviewDialog(points,parent=window)
    dialog.grab().save(str(ROOT/'docs/images/curated-v2-path-preview.png'))
    dialog.close()
    print('PASS v2: click selection, 2D preview, no-device execution, missing/stale/invalid depth, valid projection, responsive worker cancellation')

window.close()
app.processEvents()
print('PASS clean shutdown')
