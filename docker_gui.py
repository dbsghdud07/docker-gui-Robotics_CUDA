from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QComboBox, QLabel, QTextEdit, QSplitter
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from subprocess import Popen, PIPE, STDOUT
import sys, os

class ExecuteScriptThread(QThread):
    output = pyqtSignal(str)
    finishedSignal = pyqtSignal(bool)

    def __init__(self, command):
        super().__init__()
        self.command = command

    def run(self):
        process = Popen(self.command, shell=True, stdout=PIPE, stderr=STDOUT, text=True)
        success = True  # 성공 여부를 추적하는 변수

        for line in iter(process.stdout.readline, ''):
            self.output.emit(line)
            if "Error" in line:  # 오류 메시지가 포함된 경우 성공 여부를 False로 설정
                success = False

        process.stdout.close()
        returnCode = process.wait()  # 스크립트 실행 완료 대기
        if returnCode != 0:
            success = False

        self.finishedSignal.emit(success)

class DockerBuildRunGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.loadPreviousText()

    def initUI(self):
        self.setGeometry(100, 100, 800, 550)  # 너비를 800으로 조정하여 더 많은 공간 확보
        self.setWindowTitle('Docker Build and Run with Options GUI')

        mainLayout = QVBoxLayout()
        splitter = QSplitter(Qt.Horizontal)

        # Docker Build 섹션
        buildLayout = QVBoxLayout()
        buildContainer = QWidget()
        buildContainer.setLayout(buildLayout)

        self.ubuntuVersionCombo = QComboBox()
        self.ubuntuVersionCombo.addItems(["ubuntu18.04", "ubuntu20.04", "ubuntu22.04"])
        buildLayout.addWidget(QLabel("Select Ubuntu Version:"))
        buildLayout.addWidget(self.ubuntuVersionCombo)

        self.cudaVersionCombo = QComboBox()
        self.cudaVersionCombo.addItems(["10.1", "10.2", "11.0.3", "11.1.1", "11.2.2", "11.3.1", "11.4.3", "11.5.2", "11.6.2", "11.7.1", "11.8.0", "12.0.1", "12.1.1", "12.2.2", "12.3.1"])
        buildLayout.addWidget(QLabel("Select CUDA Version:"))
        buildLayout.addWidget(self.cudaVersionCombo)

        self.cudnnVersionCombo = QComboBox()
        self.cudnnVersionCombo.addItems(["cudnn7", "cudnn8", "none"])
        buildLayout.addWidget(QLabel("Select cuDNN Version:"))
        buildLayout.addWidget(self.cudnnVersionCombo)

        self.rosVersionCombo = QComboBox()
        self.rosVersionCombo.addItems(["dashing", "foxy", "humble", "none"])
        buildLayout.addWidget(QLabel("Select ROS Version:"))
        buildLayout.addWidget(self.rosVersionCombo)

        self.buildButton = QPushButton('Build Docker Image')
        self.buildButton.clicked.connect(self.buildDockerImage)
        buildLayout.addWidget(self.buildButton)

        self.output = QTextEdit()
        self.output.setPlaceholderText("Build Output...")
        self.output.setReadOnly(True)
        self.output.setStyleSheet("background-color: black; color: white;")
        buildLayout.addWidget(self.output)

        # Docker Run 섹션 (여기에 실행 관련 UI 구성 요소 추가)
        runLayout = QVBoxLayout()
        runContainer = QWidget()
        runContainer.setLayout(runLayout)

        # Docker 이미지 리스트를 불러오는 버튼
        self.loadImagesButton = QPushButton('Load Docker Images', self)
        self.loadImagesButton.clicked.connect(self.loadDockerImages)
        runLayout.addWidget(self.loadImagesButton)

        # Docker 이미지 리스트를 선택할 드롭박스 추가
        self.dockerImagesCombo = QComboBox(self)
        runLayout.addWidget(QLabel("Select Docker Image:"))
        runLayout.addWidget(self.dockerImagesCombo)

        # Docker Run 옵션 입력을 위한 QTextEdit 크기 조정
        self.dockerRunOptions = QTextEdit()
        self.dockerRunOptions.setPlaceholderText("Enter Docker run options here")
        self.dockerRunOptions.setFixedHeight(20 * 25)  # 15줄에 해당하는 높이로 설정
        runLayout.addWidget(self.dockerRunOptions)

        # 예시: Docker Run 버튼
        self.runButton = QPushButton('Run Docker Container')
        runLayout.addWidget(self.runButton)
        self.runOutput = QTextEdit()
        self.runOutput.setReadOnly(True)
        self.runOutput.setStyleSheet("background-color: black; color: white;")
        runLayout.addWidget(self.runOutput)
        self.runButton.clicked.connect(self.runDockerContainer)

        initialText = """
        <p><b>상단에 옵션을 입력해주세요</b></p>
        <p><b>-it</b> : 사용자가 입출력하는 상태 (/bin/bash)</p>
        <p><b>--privileged</b> : 시스템의 주요 자원을 모두 허용 (Computing resources)</p>
        <p><b>-e DISPLAY=$DISPLAY --env="QT_X11_NO_MITSHM=1" -v /tmp/.X11-unix:/tmp/.X11-unix:ro</b> : 화면 연결</p>
        <p><b>-v</b> : 마운트할 파일 &lt;로컬 디렉토리&gt;:&lt;컨테이너 디렉토리&gt; Local Env와 Docker Env 사이의 데이터를 특정 폴더를 통해 공유할 수 있다.)</p>
        <p><b>--gpus</b> : gpu 사용 여부</p>
        <p><b>--name</b> : 컨테이너 이름</p>
        <p><b>-p</b> : 도커에게 개방될 포트 포워딩</p>
        <p><b>-p 80:8080</b>은 컨테이너의 포트 8080을, local의 포트 80에 매핑한다.</p>
        <p><b>--network host</b> : Local computer와 Virtual computer의 인터넷 포트를 동일하게 공유한다. (기본값 : 비 공유, 포트포워딩 작업 필요)</p>
        <p><b>--ipc</b> : host의 메모리를 얼마나 할당할 것인지에 대한 메뉴 ex)--ipc=host </p>
        <p><b>Warning</b> -d 옵션을 입력하지 말것. gui 환경상 항상 백그라운드로 켜지도록 설정되어있음. </p>
        """

        # 안내 레이블 추가
        self.placeholderLabel = QLabel(initialText, self.runOutput)
        self.placeholderLabel.setStyleSheet("color: white; background-color: transparent;")
        self.placeholderLabel.move(5, 5)  # 레이블 위치 조정
        self.placeholderLabel.setAttribute(Qt.WA_TransparentForMouseEvents)  # 마우스 이벤트를 QTextEdit로 통과시킴

        # QTextEdit의 텍스트 변경 감지하여 사용자 입력 시 초기 텍스트 클리어
        self.dockerRunOptions.textChanged.connect(self.clearInitialText)

        # QTextEdit의 텍스트 변경 감지
        self.runOutput.textChanged.connect(self.checkText)

        # 컨테이너 목록을 표시할 드롭박스 추가
        self.containerCombo = QComboBox(self)
        self.loadContainersButton = QPushButton('Load Containers', self)
        self.loadContainersButton.clicked.connect(self.loadContainers)
        self.stopAndRemoveButton = QPushButton('Stop and Remove Container', self)
        self.stopAndRemoveButton.clicked.connect(self.stopAndRemoveContainer)
        self.stopAndRemoveButton.setStyleSheet("""
            QPushButton {
                background-color: red;
                color: white;
                border-style: outset;
                border-width: 4px;
                border-radius: 1px;
                border-color: beige;
                font: bold 25px;
                min-width: 10em;
                padding: 6px;
            }
            QPushButton:pressed {
                background-color: darkred;
                border-style: inset;
            }
        """)

        # 레이아웃에 위젯 추가
        runLayout.addWidget(QLabel("Select Container:"))
        runLayout.addWidget(self.containerCombo)
        runLayout.addWidget(self.loadContainersButton)
        runLayout.addWidget(self.stopAndRemoveButton)

        # Splitter에 위젯 추가
        splitter.addWidget(buildContainer)
        splitter.addWidget(runContainer)
        splitter.setSizes([400,400])  # 초기 분할 크기 설정

        mainLayout.addWidget(splitter)
        self.setLayout(mainLayout)

    def buildDockerImage(self):
        ubuntu_version = self.ubuntuVersionCombo.currentText()
        cuda_version = self.cudaVersionCombo.currentText()
        cudnn_version = self.cudnnVersionCombo.currentText()
        ros_version = self.rosVersionCombo.currentText()

        # 선택한 옵션들을 하나의 문자열로 조합
        options = f"{ubuntu_version} {cuda_version} {cudnn_version} {ros_version}"

        # build.sh 스크립트 경로 (절대 경로 또는 상대 경로로 수정하세요)
        script_path = "./gui_builder.sh"
        
        # 실행할 명령어
        cmd = f"{script_path} '{options}'"

        # 스크립트 실행을 위한 스레드 생성 및 시작
        self.executeThread = ExecuteScriptThread(cmd)
        self.executeThread.output.connect(self.updateOutput)
        self.executeThread.start()

    def runDockerContainer(self):
        selectedImageName = self.dockerImagesCombo.currentText()
        options = self.dockerRunOptions.toPlainText()  # 사용자 입력 옵션 가져오기

        # run.sh 스크립트 경로 (절대 경로 또는 상대 경로로 수정하세요)
        script_path = "./run_docker.sh"
        
        # 실행할 명령어
        cmd = f"{script_path} '{selectedImageName}' {options}"

        # 스크립트 실행을 위한 스레드 생성 및 시작
        self.executeThread = ExecuteScriptThread(cmd)
        self.executeThread.output.connect(self.updateRunOutput)
        self.executeThread.finishedSignal.connect(self.handleScriptFinished)  # 스크립트 완료 처리
        self.executeThread.start()

    def updateOutput(self, text):
        self.output.append(text)

    def handleScriptFinished(self, success):
        # 스크립트 실행 완료 후 성공 여부에 따라 메시지 처리
        if success:
            self.updateRunOutput("Successfully Run Container")
        else:
            self.updateRunOutput("Failed to run container. Check the output for errors.")

    def updateImagesDropdown(self, text):
        # 드롭박스에 이미지 리스트를 추가
        # 실행 결과가 한 줄씩 전달되므로, 각 줄을 처리
        if text.strip():  # 공백 라인이 아닌 경우에만 추가
            self.dockerImagesCombo.addItem(text.strip())

    def updateRunOutput(self, text):
        self.runOutput.append(text)

    def loadDockerImages(self):
        self.dockerImagesCombo.clear()
        # Docker 이미지 리스트를 불러오는 명령어 실행, 출력 형식을 조정함
        cmd = "docker images --format '{{.Repository}}:{{.Tag}}'"
        self.executeThread = ExecuteScriptThread(cmd)
        self.executeThread.output.connect(self.updateImagesDropdown)
        self.executeThread.start()

    def checkText(self):
        # 텍스트가 있으면 레이블을 숨기고, 없으면 레이블을 표시
        if self.runOutput.toPlainText():
            self.placeholderLabel.hide()
        else:
            self.placeholderLabel.show()

    def clearInitialText(self):
        # 사용자가 입력을 시작하면 초기 텍스트 클리어
        # 초기 텍스트가 이미 클리어된 상태라면 이 메소드는 아무 작업도 수행하지 않음
        if self.dockerRunOptions.toPlainText().strip() != "":
            # 사용자 입력이 감지되면 textChanged 시그널과의 연결을 끊어 더 이상 클리어하지 않도록 함
            self.dockerRunOptions.textChanged.disconnect(self.clearInitialText)

    def closeEvent(self, event):
        """프로그램 종료 시 사용자 입력을 파일에 저장합니다."""
        with open('previous_input.txt', 'w') as file:
            file.write(self.dockerRunOptions.toPlainText())
        event.accept()

    def loadPreviousText(self):
        """프로그램 시작 시 이전에 저장된 텍스트를 불러옵니다."""
        if os.path.exists('previous_input.txt'):
            with open('previous_input.txt', 'r') as file:
                self.dockerRunOptions.setPlainText(file.read())

    def loadContainers(self):
        self.containerCombo.clear()
        cmd = "docker ps -a --format '{{.Names}}'"
        process = Popen(cmd, shell=True, stdout=PIPE, stderr=STDOUT, executable='/bin/bash', text=True)
        for line in process.stdout:
            self.containerCombo.addItem(line.strip())
        process.stdout.close()
        process.wait()

    def stopAndRemoveContainer(self):
        selectedContainer = self.containerCombo.currentText()
        if selectedContainer:
            stopCmd = f"docker stop {selectedContainer}"
            removeCmd = f"docker rm {selectedContainer}"
            Popen(stopCmd, shell=True, stdout=PIPE, stderr=STDOUT, executable='/bin/bash')
            Popen(removeCmd, shell=True, stdout=PIPE, stderr=STDOUT, executable='/bin/bash')
            self.loadContainers()  # 컨테이너 목록 다시 불러오기

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = DockerBuildRunGUI()
    ex.showMaximized()
    sys.exit(app.exec_())
