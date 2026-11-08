"""HM 关节伺服 PCAN 调参工具入口。

用法:
    python main.py          # 连接 PCAN-USB
    python main.py --sim    # 仿真总线（无硬件），启动后自动连接
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser(description="HM 关节伺服 PCAN 调参工具")
    ap.add_argument("--sim", action="store_true", help="使用仿真总线并自动连接")
    args = ap.parse_args()

    import pyqtgraph as pg
    from PySide6.QtWidgets import QApplication

    from ui.main_window import MainWindow

    pg.setConfigOptions(antialias=False, foreground="k")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = MainWindow()
    win.show()
    if args.sim:
        win.conn.select_sim()
        win.connect_bus(win.conn.params())
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
