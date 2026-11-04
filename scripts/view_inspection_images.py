#!/usr/bin/env python3
"""Export retained/live inspection images and build a local HTML viewer."""

import argparse
import html
from pathlib import Path
import time

import cv2
import numpy as np


def save_report(report, directory):
    """Keep encoded originals unchanged; normalize only a separate preview."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = report.header.stamp
    # A simulator restart can repeat source time and site IDs. Preserve both exports.
    name = f'report_{stamp.sec}_{stamp.nanosec:09d}_site_{report.site_id}_{time.time_ns()}'
    cards = []
    for label, evidence, extension in (
        ('RGB', report.rgb_evidence, 'jpg'),
        ('Thermal', report.thermal_evidence, 'png'),
    ):
        if not evidence.valid or not evidence.image.data:
            cards.append(f'<section><h2>{label}</h2><p>Unavailable: '
                         f'{html.escape(evidence.status)}</p></section>')
            continue
        data = bytes(evidence.image.data)
        original = directory / f'{name}_{label.lower()}.{extension}'
        original.write_bytes(data)
        display = original
        if label == 'Thermal':
            pixels = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_UNCHANGED)
            if pixels is not None and pixels.ndim == 2:
                scaled = cv2.normalize(pixels, None, 0, 255, cv2.NORM_MINMAX,
                                       dtype=cv2.CV_8U)
                preview = cv2.applyColorMap(scaled, cv2.COLORMAP_INFERNO)
                display = directory / f'{name}_thermal_preview.png'
                if not cv2.imwrite(str(display), preview):
                    raise OSError(f'Cannot save {display}')
        source = evidence.image.header.stamp
        note = ('Contrast-scaled preview; original samples are unchanged. '
                'Colours do not represent calibrated temperatures.' if label == 'Thermal'
                else 'Original report JPEG.')
        cards.append(
            f'<section><h2>{label}</h2><img src="{display.name}"><p>{note}</p>'
            f'<p>Source: {source.sec}.{source.nanosec:09d}; '
            f'encoding: {html.escape(evidence.original_encoding)}</p>'
            f'<p><a href="{original.name}" download>Download original</a></p></section>')
    page = directory / f'{name}.html'
    page.write_text(
        '<!doctype html><meta charset="utf-8"><title>Inspection images</title>'
        '<style>body{font:16px sans-serif;margin:24px}main{display:flex;gap:24px;'
        'flex-wrap:wrap}section{flex:1;min-width:280px}img{max-width:100%;'
        'max-height:75vh}p{overflow-wrap:anywhere}</style>'
        f'<h1>Inspection site {report.site_id}</h1>'
        f'<p>Fire found: {report.fire_found}. {html.escape(report.status)}</p>'
        f'<p>Image evidence: {html.escape(report.evidence_status)}</p>'
        f'<main>{"".join(cards)}</main>', encoding='utf-8')
    links = ''.join(f'<li><a href="{p.name}">{p.stem}</a></li>'
                    for p in sorted(directory.glob('report_*.html'),
                                    key=lambda p: p.stat().st_mtime, reverse=True))
    (directory / 'index.html').write_text(
        '<!doctype html><meta charset="utf-8"><title>Inspection reports</title>'
        f'<h1>Inspection reports</h1><ul>{links}</ul>', encoding='utf-8')
    return page


def main():
    """Listen to completed reports without accessing vehicle command interfaces."""
    from kv_executive.msg import InspectionReport
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/inspection_reports')
    parser.add_argument('--output', default='/tmp/kv_inspection_images')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('inspection_images_exporter')
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def receive(report):
        try:
            page = save_report(report, args.output)
            print(f'Saved: {page}', flush=True)
            print(f'Open in your browser: {Path(args.output).resolve() / "index.html"}',
                  flush=True)
        except (OSError, cv2.error) as error:
            node.get_logger().error(str(error))

    subscription = node.create_subscription(InspectionReport, args.topic, receive, qos)
    print(f'Waiting for retained/new reports on {args.topic}. Stop with Ctrl+C.', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
