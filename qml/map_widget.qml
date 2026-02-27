import QtQuick
import QtLocation
import QtPositioning

Item {
    width: 350
    height: 350

    Plugin {
        id: mapPlugin
        name: "osm"
        PluginParameter {
            name: "osm.mapping.providersrepository.disabled"
            value: "true"
        }

        PluginParameter {
            name: "osm.mapping.custom.host"
            value: "https://tile.openstreetmap.org/%z/%x/%y.png"
        }
    }
    Map {
        id: map
        anchors.fill: parent
        plugin: mapPlugin
        WheelHandler {
            id: wheel
            // workaround for QTBUG-87646 / QTBUG-112394 / QTBUG-112432:
            // Magic Mouse pretends to be a trackpad but doesn't work with PinchHandler
            // and we don't yet distinguish mice and trackpads on Wayland either
            acceptedDevices: Qt.platform.pluginName === "cocoa" || Qt.platform.pluginName === "wayland"
                             ? PointerDevice.Mouse | PointerDevice.TouchPad
                             : PointerDevice.Mouse
            rotationScale: 1/120
            property: "zoomLevel"
        }
        center: QtPositioning.coordinate(39.92, 41.19) // FIXME: I added my dorm coordinates, it would be better to use place where we will use uav
        DragHandler {
            id: drag
            target: null
            onTranslationChanged: (delta) => map.pan(-delta.x, -delta.y)
        }
        Shortcut {
            enabled: map.zoomLevel < map.maximumZoomLevel
            sequences: [StandardKey.ZoomIn]
            onActivated: map.zoomLevel = Math.round(map.zoomLevel + 1)
        }
        Shortcut {
            enabled: map.zoomLevel > map.minimumZoomLevel
            sequences: [StandardKey.ZoomOut]
            onActivated: map.zoomLevel = Math.round(map.zoomLevel - 1)
        }
        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton | Qt.RightButton
            onClicked: (mouse)=> {
                mouseInputHandler.handle_mouse_input(mouse.button, map.toCoordinate(mapToItem(parent, mouse.x, mouse.y)))
            }
        }
        MapItemView{
            model: datamodel
            delegate: MapQuickItem{
                id: item
                // begin configuration
                property var position: model.position
                property var plane_type: model.plane_type
                onPositionChanged: restart();
                function restart(){ // FIXME: IDK how this thing even works. I need to do more research
                    anim.stop()
                    anim.from = position
                    anim.to = position
                    anim.start()
                }
                CoordinateAnimation {
                    id: anim
                    target: item
                    duration: 60 * 1000
                    property: "coordinate"
                }
                // end of configuration
                anchorPoint.x: plane_image.width/2
                anchorPoint.y: plane_image.height/2
                sourceItem: Image {
                    id: plane_image
                    function getImage() {
                        if (plane_type === 0) {
                            return "../ui_files/blue_plane.png"
                        } else if (plane_type === 1) {
                            return "../ui_files/red_plane.png"
                        } else if (plane_type === 2) {
                            return "../ui_files/green_plane.png"
                        } else if (plane_type === 3) {
                            return "../ui_files/yellow_plane.png"
                        }
                        return "ErrorNoPlaneImageForPlaneType"
                    }
                    // TODO: add rotation
                    source: getImage()
                    width: 20
                    height: 20
                }
            }
        }
    }
}