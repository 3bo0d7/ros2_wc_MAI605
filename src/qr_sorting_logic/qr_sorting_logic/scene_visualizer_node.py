import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class SceneVisualizer(Node):
    """Publishes a compact RViz workcell view for the QR sorting demo."""

    def __init__(self):
        super().__init__('scene_visualizer_node')

        self.declare_parameter('frame_id', 'panda_link0')
        self.declare_parameter('bin_a.pose', [0.30, 0.50, 0.35, 0.0, 1.0, 0.0, 0.0])
        self.declare_parameter('bin_b.pose', [0.50, 0.28, 0.35, 0.0, 1.0, 0.0, 0.0])
        self.declare_parameter('bin_c.pose', [0.30, -0.50, 0.35, 0.0, 1.0, 0.0, 0.0])
        self.declare_parameter('reject_bin.pose', [0.50, -0.28, 0.35, 0.0, 1.0, 0.0, 0.0])
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.selected_bin = ''
        self.qr_text = 'Waiting'
        self.object_picked = False
        self.bin_place_z_offset = 0.04
        self.bins = self.load_bins()

        self.create_subscription(String, '/qr_sort/bin_id', self.on_bin_id, 10)
        self.create_subscription(String, '/qr_sort/sim_camera/qr_text', self.on_qr_text, 10)
        self.create_subscription(Bool, '/qr_sort/scan_ready', self.on_scan_ready, 10)
        self.pub = self.create_publisher(MarkerArray, '/qr_sort/visualization', 10)
        self.timer = self.create_timer(0.5, self.publish_scene)

        self.get_logger().info(
            'Scene visualizer ready. Add MarkerArray topic /qr_sort/visualization in RViz.'
        )

    def load_bins(self):
        return {
            'bin_a': {
                'label': 'BIN A',
                'pos': self.pose_position('bin_a.pose'),
                'color': [0.0, 0.85, 0.2, 0.58],
            },
            'bin_b': {
                'label': 'BIN B',
                'pos': self.pose_position('bin_b.pose'),
                'color': [0.2, 0.4, 1.0, 0.58],
            },
            'bin_c': {
                'label': 'BIN C',
                'pos': self.pose_position('bin_c.pose'),
                'color': [1.0, 0.8, 0.0, 0.58],
            },
            'reject_bin': {
                'label': 'REJECT',
                'pos': self.pose_position('reject_bin.pose'),
                'color': [1.0, 0.0, 0.0, 0.58],
            },
        }

    def pose_position(self, parameter_name):
        raw = list(self.get_parameter(parameter_name).value)
        return [float(raw[0]), float(raw[1]), float(raw[2])]

    def on_bin_id(self, msg):
        if msg.data != self.selected_bin:
            self.get_logger().info(f'Selected bin visualized: {msg.data}')
        self.selected_bin = msg.data

    def on_qr_text(self, msg):
        self.qr_text = msg.data

    def on_scan_ready(self, msg):
        if msg.data:
            self.object_picked = True

    def make_marker(self, marker_id, marker_type, ns, position, scale, color, text=''):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])
        marker.text = text
        return marker

    def label(self, marker_id, text, position, size=0.035):
        return self.make_marker(
            marker_id,
            Marker.TEXT_VIEW_FACING,
            'labels',
            position,
            [size, size, size],
            [0.95, 0.95, 0.95, 1.0],
            text,
        )

    def point(self, xyz):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        return p

    def line_marker(self, marker_id, ns, points, color, width=0.01):
        marker = self.make_marker(
            marker_id,
            Marker.LINE_STRIP,
            ns,
            [0.0, 0.0, 0.0],
            [width, 0.0, 0.0],
            color,
        )
        marker.points = [self.point(p) for p in points]
        return marker

    def line_list_marker(self, marker_id, ns, point_pairs, color, width=0.006):
        marker = self.make_marker(
            marker_id,
            Marker.LINE_LIST,
            ns,
            [0.0, 0.0, 0.0],
            [width, 0.0, 0.0],
            color,
        )
        marker.points = [self.point(p) for pair in point_pairs for p in pair]
        return marker

    def add_qr_panel(self, markers, marker_id, object_center):
        # A small QR-like panel on the workpiece makes the scan target obvious in RViz.
        panel_center = [object_center[0], object_center[1] - 0.034, object_center[2] + 0.005]
        markers.markers.append(self.make_marker(
            marker_id, Marker.CUBE, 'qr_panel',
            panel_center,
            [0.048, 0.003, 0.048],
            [1.0, 1.0, 1.0, 1.0],
        ))
        marker_id += 1

        offsets = [
            [-0.014, 0.0, 0.014],
            [0.012, 0.0, 0.014],
            [-0.014, 0.0, -0.012],
            [0.004, 0.0, -0.004],
            [0.016, 0.0, -0.016],
        ]
        for dx, dy, dz in offsets:
            markers.markers.append(self.make_marker(
                marker_id, Marker.CUBE, 'qr_panel',
                [panel_center[0] + dx, panel_center[1] + dy - 0.002, panel_center[2] + dz],
                [0.010, 0.003, 0.010],
                [0.0, 0.0, 0.0, 1.0],
            ))
            marker_id += 1

        return marker_id

    def add_open_bin(self, markers, marker_id, bin_id, info, selected):
        pos = info['pos']
        color = info['color']
        outer = 0.20
        wall = 0.010
        floor = 0.012
        height = 0.10
        bottom_z = pos[2] - 0.055
        wall_z = bottom_z + floor + (height * 0.5)
        top_z = bottom_z + floor + height
        alpha = 0.90 if selected else 0.46
        bin_color = [color[0], color[1], color[2], alpha]
        rim_color = [min(color[0] + 0.15, 1.0), min(color[1] + 0.15, 1.0), min(color[2] + 0.15, 1.0), min(alpha + 0.08, 1.0)]

        pieces = [
            ([pos[0], pos[1], bottom_z + floor * 0.5], [outer, outer, floor], bin_color),
            ([pos[0] + outer * 0.5 - wall * 0.5, pos[1], wall_z], [wall, outer, height], bin_color),
            ([pos[0] - outer * 0.5 + wall * 0.5, pos[1], wall_z], [wall, outer, height], bin_color),
            ([pos[0], pos[1] + outer * 0.5 - wall * 0.5, wall_z], [outer, wall, height], bin_color),
            ([pos[0], pos[1] - outer * 0.5 + wall * 0.5, wall_z], [outer, wall, height], bin_color),
            ([pos[0] + outer * 0.5, pos[1], top_z + wall * 0.5], [wall, outer + wall, wall], rim_color),
            ([pos[0] - outer * 0.5, pos[1], top_z + wall * 0.5], [wall, outer + wall, wall], rim_color),
            ([pos[0], pos[1] + outer * 0.5, top_z + wall * 0.5], [outer + wall, wall, wall], rim_color),
            ([pos[0], pos[1] - outer * 0.5, top_z + wall * 0.5], [outer + wall, wall, wall], rim_color),
        ]

        for piece_pos, scale, piece_color in pieces:
            markers.markers.append(self.make_marker(
                marker_id, Marker.CUBE, f'{bin_id}_open_bin', piece_pos, scale, piece_color
            ))
            marker_id += 1

        return marker_id, bottom_z, top_z

    def publish_scene(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        marker_id = 1

        table_center = [0.45, 0.0, 0.30]
        pick_pos = [0.45, 0.0, 0.35]
        scan_pos = [0.35, -0.25, 0.50]
        camera_pos = [0.35, -0.25, 0.62]

        markers.markers.append(self.make_marker(
            marker_id, Marker.CUBE, 'table',
            table_center,
            [0.85, 0.75, 0.02],
            [0.30, 0.30, 0.30, 0.28],
        ))
        marker_id += 1

        markers.markers.append(self.make_marker(
            marker_id, Marker.CUBE, 'pick_support',
            [0.45, 0.0, 0.30],
            [0.16, 0.16, 0.024],
            [0.42, 0.42, 0.42, 0.75],
        ))
        marker_id += 1

        if not self.object_picked:
            markers.markers.append(self.make_marker(
                marker_id, Marker.CUBE, 'object',
                pick_pos,
                [0.06, 0.06, 0.06],
                [1.0, 0.55, 0.0, 1.0],
            ))
            marker_id += 1
            marker_id = self.add_qr_panel(markers, marker_id, pick_pos)

        markers.markers.append(self.make_marker(
            marker_id, Marker.CUBE, 'scanner',
            scan_pos,
            [0.13, 0.05, 0.08],
            [0.0, 0.55, 1.0, 0.85],
        ))
        marker_id += 1

        markers.markers.append(self.make_marker(
            marker_id, Marker.CUBE, 'camera',
            camera_pos,
            [0.08, 0.05, 0.05],
            [0.05, 0.05, 0.05, 1.0],
        ))
        marker_id += 1

        frustum = [
            (camera_pos, [0.29, -0.19, 0.51]),
            (camera_pos, [0.41, -0.19, 0.51]),
            (camera_pos, [0.29, -0.31, 0.51]),
            (camera_pos, [0.41, -0.31, 0.51]),
        ]
        markers.markers.append(self.line_list_marker(
            marker_id, 'camera_frustum', frustum, [0.0, 0.8, 1.0, 0.75], width=0.004
        ))
        marker_id += 1

        bin_geometry = {}
        for bin_id, info in self.bins.items():
            selected = bin_id == self.selected_bin
            pos = info['pos']
            marker_id, bottom_z, top_z = self.add_open_bin(markers, marker_id, bin_id, info, selected)
            bin_geometry[bin_id] = {'bottom_z': bottom_z, 'top_z': top_z}

            markers.markers.append(self.label(
                marker_id,
                info['label'] + (' *' if selected else ''),
                [pos[0], pos[1], top_z + 0.055],
                size=0.032,
            ))
            marker_id += 1

        route_points = [
            [0.45, 0.0, 0.47],
            [0.35, -0.25, 0.56],
        ]
        if self.selected_bin in self.bins:
            bp = self.bins[self.selected_bin]['pos']
            top_z = bin_geometry[self.selected_bin]['top_z']
            route_points.append([bp[0], bp[1], top_z + 0.07])
        markers.markers.append(self.line_marker(
            marker_id, 'route', route_points, [1.0, 1.0, 1.0, 0.80], width=0.008
        ))
        marker_id += 1

        if self.object_picked and self.selected_bin in self.bins:
            bp = self.bins[self.selected_bin]['pos']
            placed_pos = [bp[0], bp[1], bp[2] + self.bin_place_z_offset]
            markers.markers.append(self.make_marker(
                marker_id, Marker.CUBE, 'placed_object',
                placed_pos,
                [0.06, 0.06, 0.06],
                [1.0, 0.55, 0.0, 0.88],
            ))
            marker_id += 1
            marker_id = self.add_qr_panel(markers, marker_id, placed_pos)

        status = f'QR: {self.qr_text}  ->  {self.selected_bin or "waiting"}'
        markers.markers.append(self.label(
            marker_id,
            status,
            [0.15, -0.58, 0.78],
            size=0.035,
        ))
        marker_id += 1

        markers.markers.append(self.label(marker_id, 'PICK', [0.55, 0.04, 0.42], size=0.030))
        marker_id += 1
        markers.markers.append(self.label(marker_id, 'SCAN CAMERA', [0.20, -0.42, 0.66], size=0.030))

        self.pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SceneVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
