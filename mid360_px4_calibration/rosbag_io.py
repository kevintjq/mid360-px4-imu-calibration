from __future__ import annotations

from collections import Counter
from typing import Dict, Iterable, List

import numpy as np

from .io import sanitize_samples
from .types import ImuSamples


def load_imu_topics_from_rosbag(
    bag_path: str,
    topics: Iterable[str],
    stamp_source: str = "header",
) -> Dict[str, ImuSamples]:
    try:
        import rosbag
    except ImportError as exc:
        raise RuntimeError("failed to import rosbag; run inside a sourced ROS1/Noetic environment") from exc

    topic_list = list(topics)
    storage = {
        topic: {
            "t": [],
            "gyro": [],
            "accel": [],
            "frames": Counter(),
        }
        for topic in topic_list
    }
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, bag_t in bag.read_messages(topics=topic_list):
            stamp = msg.header.stamp if stamp_source == "header" else bag_t
            storage[topic]["t"].append(stamp.to_sec())
            storage[topic]["gyro"].append(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ]
            )
            storage[topic]["accel"].append(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]
            )
            storage[topic]["frames"][msg.header.frame_id] += 1

    out: Dict[str, ImuSamples] = {}
    missing: List[str] = []
    for topic in topic_list:
        if not storage[topic]["t"]:
            missing.append(topic)
            continue
        frame_id = ""
        if storage[topic]["frames"]:
            frame_id = storage[topic]["frames"].most_common(1)[0][0]
        out[topic] = sanitize_samples(
            ImuSamples(
                np.asarray(storage[topic]["t"], dtype=float),
                np.asarray(storage[topic]["gyro"], dtype=float),
                np.asarray(storage[topic]["accel"], dtype=float),
                frame_id=frame_id,
            )
        )
    if missing:
        raise ValueError("no IMU messages found for topic(s): {}".format(", ".join(missing)))
    return out
