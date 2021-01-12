from PyQt5.QtMultimedia import QMediaContent, QMediaPlayer
from PyQt5.QtWidgets import QFileDialog, QApplication
from PyQt5 import uic

class videoPlayer:
    def __init__(self):
        self.ui = uic.loadUi('bluerov.ui') 
        self.player = QMediaPlayer()
        self.player.setVideoOutput(self.ui.videoWidget)
        self.ui.playPushButton.clicked.connect(self.openVideoFile)

    def openVideoFile(self):
        self.player.setMedia(QMediaContent(QFileDialog.getOpenFileUrl()[0]))
        self.player.play()


if __name__ == "__main__":
    app = QApplication([])
    myPlayer = videoPlayer()
    myPlayer.ui.show()
    app.exec()