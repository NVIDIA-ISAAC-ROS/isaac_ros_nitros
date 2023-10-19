// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2018-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "engine/gems/serialization/json.hpp"

namespace nvidia {
namespace isaac {
namespace sight {
namespace named_sop_details {

// Truncate all keys (including nested keys) to single letter abbreviations.
// Limitations of current implementation:
//   (1) Keys consisting entirely of numerical values will not be truncated.
//   (2) Truncation that results in changing a key to "0" will cause that key to be interpreted as a
//       array index, likely causing undesired behaviour. JSON containing the key "0" cannot be
//       reversibly flattened using nlohmann/json (i.e., my_json.flatten().unflatten() != my_json)
//   (3) If two keys at the same level are truncated to the same letter, their contents will be
//       merged (i.e., the contents of the second object will append or overwrite the first)
//   (4) Empty strings and arrays will be transformed into a null object.
// These limitations are demonstrated in the associated unit tests (json_key_truncation.cpp)
void TruncateJsonKeys(Json& json);

}  // namespace named_sop_details

// Converts JSON in "named SOP" format to "standard SOP" format. The output "sop" is
// overwritten by converted input "named_sop". The supported input JSON format is shown below:
//
//    "named_sop_1": {
//      SOP_1_DATA
//    },
//      "named_sop_2": {
//      SOP_2_DATA
//    },
//
//    ...
//
//    "named_sop_N": {
//      SOP_N_DATA
//    }
//
// Each DATA block must adhere to the standard SOP format, with two exceptions:
//   (1) The outermost "data": [{}] surrounding each SOP is not required.
//   (2) Full words may be used for each attribute (e.g., "type" instead of "t"). All keys will
//       be automatically truncated to a single letter abbreviation upon conversion from JSON to
//       SOP.
//
// The example JSON shown above will be converted to a single SOP with the following format:
//
//    {
//      "type": "sop",
//      "data": [
//        {
//          "type": "sop",
//          "data": [
//            {
//              SOP_1_DATA
//            }
//          ]
//        },
//        {
//          "type": "sop",
//          "data": [
//            {
//              SOP_2_DATA
//            }
//          ]
//        },
//
//        ...
//
//        {
//          "type": "sop",
//          "data": [
//            {
//              SOP_N_DATA
//            }
//          ]
//        }
//      ]
//    }
//
// NOTE: In the final conversion, keys will all be abbreviated to single letters.
void ConvertFromNamedSop(const Json& named_sop, Json& sop);

}  // namespace sight
}  // namespace isaac
}  // namespace nvidia
