# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""One-off: bake the front-camera optical-frame roll into a recording's tf.

`SpotHighLevel` rights the sideways front images but leaves their optical tf
frames at the raw mount orientation, so recorded tf is wrong for depth
back-projection. This copies a recording and rolls the front optical frames in
its `tf` stream so the baked data matches the upright images — proving we can
drop the render-time roll once new recordings store correct tf.

Usage:
    python demo_fix_spot_recording_tf.py <source.db> <fixed.db>
"""

from __future__ import annotations

from pathlib import Path
import shutil
import sqlite3
import sys

from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.bosdyn.spot.config import (
    FRONT_CAMERA_MIRROR_HALF_TURN,
    FRONT_CAMERA_ROTATE_UPRIGHT,
)
from dimos.robot.bosdyn.spot.utils import roll_optical_frame

FRONT_OPTICAL_FRAME_ROLLS = {
    "frontleft_camera_optical": FRONT_CAMERA_ROTATE_UPRIGHT,
    "frontright_camera_optical": FRONT_CAMERA_ROTATE_UPRIGHT + FRONT_CAMERA_MIRROR_HALF_TURN,
}


def main() -> None:
    source = Path(sys.argv[1]).expanduser()
    target = Path(sys.argv[2]).expanduser()
    if target.exists():
        raise FileExistsError(f"{target} already exists — refusing to overwrite")

    print(f"Copying {source} -> {target}")
    shutil.copyfile(source, target)

    connection = sqlite3.connect(str(target))
    rows = connection.execute("SELECT id, data FROM tf_blob ORDER BY id").fetchall()
    rolled = 0
    for blob_id, data in rows:
        message = TFMessage.lcm_decode(bytes(data))
        transform = message.transforms[0]
        quarter_turns = FRONT_OPTICAL_FRAME_ROLLS.get(transform.child_frame_id, 0)
        if not quarter_turns:
            continue
        fixed = TFMessage(roll_optical_frame(transform, quarter_turns))
        connection.execute(
            "UPDATE tf_blob SET data = ? WHERE id = ?", (fixed.lcm_encode(), blob_id)
        )
        rolled += 1

    connection.commit()
    connection.close()
    print(f"Rolled {rolled} front-camera tf records out of {len(rows)} total")


if __name__ == "__main__":
    main()
