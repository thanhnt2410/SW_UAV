import sys
from PyQt5.QtWidgets import QApplication, QMainWindow

app = QApplication(sys.argv)

w = QMainWindow()
w.resize(800, 600)
w.show()

sys.exit(app.exec_())