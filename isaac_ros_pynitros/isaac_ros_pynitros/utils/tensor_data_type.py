# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0


class TensorDataTypeUtils():
    """
    Helper class to provide utility functions for tensor data types.

    Use the following int values to represent tensor data types:
    -  1: "int8"
    -  2: "uint8"
    -  3: "int16"
    -  4: "uint16"
    -  5: "int32"
    -  6: "uint32"
    -  7: "int64"
    -  8: "uint64"
    -  9: "float32"
    - 10: "float64"
    """

    data_type_map = {
        1: ('|i1', 1),
        2: ('|u1', 1),
        3: ('|i2', 2),
        4: ('|u2', 2),
        5: ('|i4', 4),
        6: ('|u4', 4),
        7: ('|i8', 8),
        8: ('|u8', 8),
        9: ('|f4', 4),
        10: ('|f8', 8)
    }

    @classmethod
    def get_typestr(cls, data_type):
        return cls.data_type_map[data_type][0]

    @classmethod
    def get_size_in_bytes(cls, data_type):
        return cls.data_type_map[data_type][1]
