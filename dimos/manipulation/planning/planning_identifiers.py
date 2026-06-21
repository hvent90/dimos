# Copyright 2025-2026 Dimensional Inc.
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

"""Compatibility shim for planning-group identifier helpers.

New code should import from ``dimos.manipulation.planning.groups.identifiers``.
"""

from dimos.manipulation.planning.groups.identifiers import (
    assert_global_joint_names,
    assert_local_joint_names,
    assert_valid_local_joint_name,
    assert_valid_robot_name,
    is_global_joint_name,
    local_joint_name_from_global,
    make_global_joint_name,
    make_global_joint_names,
    make_planning_group_id,
    parse_planning_group_id,
)

__all__ = [
    "assert_global_joint_names",
    "assert_local_joint_names",
    "assert_valid_local_joint_name",
    "assert_valid_robot_name",
    "is_global_joint_name",
    "local_joint_name_from_global",
    "make_global_joint_name",
    "make_global_joint_names",
    "make_planning_group_id",
    "parse_planning_group_id",
]
