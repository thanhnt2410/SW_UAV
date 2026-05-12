from PyQt5.QtWidgets import QApplication
from PyQt5.QtWebEngineWidgets import QWebEngineView

app = QApplication([])

view = QWebEngineView()
view.setHtml("<h1>Hello</h1>")
view.show()

app.exec()