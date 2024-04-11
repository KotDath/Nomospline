import QtQuick 2.0
import QtQuick.Controls 2.0

ApplicationWindow {
    id: dialogWindow
    width: 400
    height: 200
    visible: false

    Dialog {
        id: dialog
        title: "Dialog Title"
        visible: true
        standardButtons: StandardButton.Ok | StandardButton.Cancel
        onAccepted: {
            // Handle accepted action
            dialogWindow.close()
        }
        onRejected: {
            // Handle rejected action
            dialogWindow.close()
        }
    }
}