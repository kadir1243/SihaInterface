import QtQuick
import QtQuick.Shapes
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
    }
    Map {
        id: map
        objectName: "map"
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
        TapHandler {
            acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
            onTapped: (eventPoint, button) => {
                            switch (point.modifiers) {
                                case Qt.ControlModifier:
                                    mouseInputHandler.handle_mouse_input_to_map_with_ctrl(button, map.toCoordinate(mapToItem(parent, eventPoint.position.x, eventPoint.position.y)));
                                    break;
                                default:
                                    mouseInputHandler.handle_mouse_input_to_map(button, map.toCoordinate(mapToItem(parent, eventPoint.position.x, eventPoint.position.y)), eventPoint.position.x, eventPoint.position.y);
                                    break;
                            }
            }
        }
        MapItemView {
            model: plane_data_model
            delegate: MapQuickItem{
                id: item
                // begin configuration
                property var position: model.position
                property var plane_type: model.plane_type
                property var rotation_: model.rotation // rotation is a member after qt 6.11
                coordinate: position;
                // end of configuration
                anchorPoint.x: plane_image.width/2
                anchorPoint.y: plane_image.height/2
                autoFadeIn: false
                sourceItem: Image {
                    id: plane_image
                    function getImage() {
                        if (plane_type === 2) {
                            return "../ui_files/blue_plane.svg"
                        } else if (plane_type === 1) {
                            return "../ui_files/red_plane.svg"
                        } else if (plane_type === 0) {
                            return "../ui_files/green_plane.svg"
                        } else if (plane_type === 3) {
                            return "../ui_files/yellow_plane.svg"
                        }
                        return "ErrorNoPlaneImageForPlaneType"
                    }
                    transform: Rotation {
                        angle: rotation_
                        origin.x: plane_image.width / 2
                        origin.y: plane_image.height / 2
                    }
                    source: getImage()
                    width: 32
                    height: 32
                    horizontalAlignment: Image.AlignHCenter
                    verticalAlignment: Image.AlignVCenter
                }
            }
        }
        MapItemView {
            model: coord_data_model
            delegate: MapQuickItem {
                id: item2
                // begin configuration
                property var position: model.position
                property var coord_type: model.coord_type
                coordinate: position
                // end of configuration
                anchorPoint.x: rect.width / 2
                anchorPoint.y: rect.height / 2
                sourceItem: Rectangle {
                    id: rect

                    function getColor() {
                        if (coord_type === 5 || coord_type === 6 || coord_type === 7 || coord_type === 8 || coord_type === 9) {
                            return "red"
                        } else if (coord_type === 1) {
                            return "blue"
                        }
                        return "purple"
                    }

                    color: getColor()
                    width: 20
                    height: 20
                }
            }
        }
        MapPolygon {
            property var gc1: coords_for_geofence.gc1
            property var gc2: coords_for_geofence.gc2
            property var gc3: coords_for_geofence.gc3
            property var gc4: coords_for_geofence.gc4
            id: geofence
            color: "#4032cd32"
            path: [gc1, gc2, gc3, gc4]
        }
        MapQuickItem {
            id: reposition_target
            property var reposition_target_coords: reposition_target_coord.position
            coordinate: reposition_target_coords
            anchorPoint.x: reposition_image.width / 2
            anchorPoint.y: reposition_image.height
            sourceItem: Image {
                id: reposition_image

                source: "../ui_files/target_location.svg"
                width: 32
                height: 32
            }
        }
        MapItemView {
            model: server_ads_data_model
            delegate: MapCircle {
                id: item2
                property var position: model.position
                property var size: model.size
                center: position
                opacity: 0.5
                radius: size
                color: 'red'
            }
        }
        MapItemView {
            model: user_ads_data_model
            delegate: MapCircle {
                id: item4
                property var position: model.position
                property var size: model.size
                property var is_selected: model.is_selected
                function getColor() {
                    if (is_selected) {
                        return "blue"
                    }
                    return "yellow"
                }
                center: position
                opacity: 0.5
                radius: size
                color: getColor()
            }
        }
    }
}