import QtQuick.Controls 2.15
import QtQuick 2.0
import QtQuick.Dialogs as QQD
import QtQuick.Layouts

import Scene 1.0
import WindowUtils 1.0

ApplicationWindow {
    property string windowFull: "FullScreen"
    property string windowWindowed: "Windowed"

    property bool isDebug: true

    function getDirMesh() {
        if (theWindow.isDebug) {
            return windowUtils.getDefaultSplineFolder();
        } else {
            return windowUtils.getHomeDirectory();
        }
    }

    function getDirBREP() {
        if (theWindow.isDebug) {
            return windowUtils.getDefaultBREPFolder();
        } else {
            return windowUtils.getHomeDirectory();
        }
    }

    title: "Qt Quick Controls Gallery"
    id: theWindow
    minimumWidth: 480
    minimumHeight: 360
    visibility: windowWindowed
    visible: true
    width: 1366
    height: 768

    WindowUtils {
        id: windowUtils
        scene: scene
        window: theWindow
    }
    QQD.FileDialog {
        id: fileDialog

        title: "Please choose a file"
        nameFilters: ["NURBS Spline or mesh (*.obj *.d3m)"]
        currentFolder: theWindow.getDirMesh()
        fileMode: QQD.FileDialog.OpenFiles

        visible: false

        onAccepted: {
            for (let i = 0; i < fileDialog.selectedFiles.length; i++) {

                let path = fileDialog.selectedFiles[i].toString();
                path = path.replace(/^(file:\/{2})/,"");
                let cleanPath = decodeURIComponent(path);
                windowUtils.importMesh(cleanPath)
            }

        }
    }

    QQD.FileDialog {
        id: brepDialog

        title: "Please choose a file"
        nameFilters: ["BREP struct (*.json)"]
        currentFolder: theWindow.getDirBREP()
        fileMode: QQD.FileDialog.OpenFiles

        visible: false

        onAccepted: {
            for (let i = 0; i < brepDialog.selectedFiles.length; i++) {

                let path = brepDialog.selectedFiles[i].toString();
                path = path.replace(/^(file:\/{2})/,"");
                let cleanPath = decodeURIComponent(path);
                windowUtils.importBREP(cleanPath)
            }

        }
    }

    menuBar: MenuBar {
        Menu {
            width: 300
            title: qsTr("&File")
            Action {
                text: qsTr("&Import...")
                onTriggered: {
                    fileDialog.open()
                }
            }
            Action {
                text: qsTr("&Import BREP")
                onTriggered: {
                    brepDialog.open()
                }
            }
            Action {
                text: qsTr("&Clear scene")
                onTriggered: {
                    windowUtils.clear()
                }
            }
            Action {
                text: qsTr("&Evaluate")
                onTriggered: {
                    windowUtils.evaluate()
                }
            }
            Action {
                text: qsTr("&Calculate Intersection")
                onTriggered: {
                    var component = Qt.createComponent("IntersectionDialog.qml");

                    windowUtils.calculateIntersection()
                }
            }
            Action {
                text: qsTr("&Calculate Difference")
                onTriggered: {
                    var component = Qt.createComponent("IntersectionDialog.qml");

                    windowUtils.calculateDiff()
                }
            }
            MenuSeparator { }
            Action { text: qsTr("&Quit") }
        }
        Menu {
            width: 300
            title: qsTr("&Examples")
            Action {
                text: qsTr("&Neco")
                onTriggered: windowUtils.importMesh("../examples/neco.obj")
            }

            Action {
                text: qsTr("&Half Sphere")
                onTriggered: windowUtils.importMesh("../examples/spheres/sample.obj")
            }

        }
        Menu {
            width: 300
            title: qsTr("&Help")
            Action { text: qsTr("&About") }
        }
    }

    Item {
        anchors.fill: parent
        id: keyobject
        focus: true
        Keys.onPressed: {
            if (event.key === Qt.Key_F11) {
                if (theWindow.visibility === Window.FullScreen)
                    theWindow.visibility = Window.AutomaticVisibility;
                else
                    theWindow.visibility = Window.FullScreen;
            }

            if (event.key === Qt.Key_F1) {
                windowUtils.setForward()
            }
            if (event.key === Qt.Key_F2) {
                windowUtils.setBackward()
            }
            if (event.key === Qt.Key_F3) {
                windowUtils.setRight()
            }
            if (event.key === Qt.Key_F4) {
                windowUtils.setLeft()
            }
            if (event.key === Qt.Key_F5) {
                windowUtils.setUp()
            }
            if (event.key === Qt.Key_F6) {
                windowUtils.setDown()
            }
        }

        SplitView {
            anchors.fill: parent

            Settings {
                windowUtils: windowUtils
                SplitView.fillHeight: true
                SplitView.minimumWidth: 200
                SplitView.preferredWidth:parent.width / 5.0
                SplitView.maximumWidth: parent.width / 2.0
                clip: true
            }

            Scene {
                id: scene
                width: 1000
                height: 1000
            }


        }
    }
}