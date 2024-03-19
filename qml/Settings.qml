import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Dialogs

Column {
    property var windowUtils
    spacing: 5

    ColorDialog {
        id: colorDialog
        onAccepted: {
            console.log("Selected Color:", color.r, color.g, color.b, color.a)
        }
    }

    Row {
        CheckBox {
            text: "Draw meshes"
            onCheckedChanged: {
                windowUtils.drawMeshes(checked)
                if (checked) {
                    //colorDialog.visible = true
                }
            }

        }
        Button {
            text: "Edit"
        }
    }


    CheckBox {
        text: "Draw evaluated splines"
        onCheckedChanged: windowUtils.drawEvaluatedMeshes(checked)
    }

    CheckBox {
        text: "Draw splines"
        onCheckedChanged: windowUtils.drawSplines(checked)
    }

    Rectangle {
        color: "gray"
        width: 200
        height: 40

        TextInput {
            color: "white"
            id: floatInput
            width: 200
            height: 40
            validator: RegularExpressionValidator { regularExpression: /^(\d+)?(\.)?(\d+)?$/ }
            inputMethodHints: Qt.ImhFormattedNumbersOnly
        }
    }


}