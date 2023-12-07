import QtLocation 5.15
import QtPositioning 5.15
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Styles 1.4
import QtQuick.Extras 1.4
import QtQuick.Layouts 1.14
import QtQuick.Shapes 1.15
import QtQuick.Window 2.0
import QtQuick.Dialogs 1.0
import Qt.labs.location 1.0

Window {
    id : mainWindow
    visible: true
    width: 800
    height: 600
    title: qsTr("SimGUI")

    property var map_follow_vehicle: false
    property var location_history_proxy: backend.location_history
    function centerMapOn(point){
        cute_map.center = point;
        console.log("Centering map on: " + point)
    } 
    Component.onCompleted: {
        console.log("QML COMPLETE")
        backend.exitRequested.connect(mainWindow.close)
    }

    Plugin {
        id: mapPlugin
        name: "mapboxgl" // "mapboxgl", "esri", "osm"

    }

    Map {
        id: cute_map
        anchors.fill : parent
        plugin: mapPlugin
        center: QtPositioning.coordinate(32.072734, 34.787465)
        zoomLevel: 17.0
        maximumZoomLevel: 17.9
        activeMapType: supportedMapTypes[0]

        function toggleMap(){
            if (activeMapType == supportedMapTypes[0]){
                activeMapType = supportedMapTypes[1]
            }
            else{
                activeMapType = supportedMapTypes[0]
            }
        }
        MapQuickItem {
            scale: 1.0
            sourceItem: Rectangle{
                width: radius * 2
                height: radius * 2
                color: 'green'
                opacity: 0.5
                radius: 15
                border.color:  "black"
                border.width: 2
            }
            coordinate: backend.target_location
            anchorPoint: Qt.point(sourceItem.width / 2, sourceItem.height / 2)
        }

        MapItemView{
            model: mainWindow.location_history_proxy
            delegate: MapQuickItem {
                scale: 1.0
                sourceItem: Rectangle{
                    color: 'red';
                    opacity: 1.0
                    radius: 5
                    width: 2*radius
                    height: width
                }
                coordinate: modelData
                anchorPoint: Qt.point(sourceItem.width / 2, sourceItem.height / 2)
            }
        }

        MapQuickItem {
            scale: 1.0
            sourceItem: Rectangle{
                width: radius * 2
                height: radius * 2
                color: 'red'
                opacity: 1.0
                radius: 15
                border.color:  "black"
                border.width: 2
            }
            coordinate: backend.robot_location
            anchorPoint: Qt.point(sourceItem.width / 2, sourceItem.height / 2)
        }

        MouseArea {
            id: mapMouseArea
            hoverEnabled: true
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.RightButton
            onClicked: {
                if (mouse.button === Qt.LeftButton) {
                    var coord = cute_map.toCoordinate(Qt.point(mapMouseArea.mouseX, mapMouseArea.mouseY));
                    backend.set_target_point(coord.latitude, coord.longitude)
                }
                else if (mouse.button === Qt.RightButton) {
                        contextMenu.popup()
                }
            }
            Menu {
                id: contextMenu
                Action {
                    text: "Center map on the Robot"
                    onTriggered: {
                        centerMapOn(backend.robot_location)
                    }
                }
                Action {
                    text: "Send the Robot here"
                    onTriggered: {
                        var coord = cute_map.toCoordinate(Qt.point(mapMouseArea.mouseX, mapMouseArea.mouseY));
                        backend.set_target_point(coord.latitude, coord.longitude)
                    }
                }
                Action {
                    text: "Clear history"
                    onTriggered: {
                        backend.clearHistory() 
                    }
                }
                Action {
                    text: "Change Map Type"
                    onTriggered: {
                        cute_map.toggleMap() 
                    }
                }
            }
        }
    }
}
