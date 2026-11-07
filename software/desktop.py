"""Qt binding-neutral font initialization for Windows and headless checks."""
from pathlib import Path


def setup_fonts(app, database, font_class):
    families=[]
    for name in ('msyh.ttc','segoeui.ttf'):
        path=Path('C:/Windows/Fonts')/name
        if path.exists():
            identifier=database.addApplicationFont(str(path))
            if identifier >= 0:
                families.extend(database.applicationFontFamilies(identifier))
    if families:
        app.setFont(font_class(families[0],10))
