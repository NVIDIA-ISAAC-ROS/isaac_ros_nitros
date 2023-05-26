/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/gems/serialization/json.hpp"

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
