/*
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef NVIDIA_GXF_STD_METRIC_HPP_
#define NVIDIA_GXF_STD_METRIC_HPP_

#include <string>
#include <vector>

#include "gxf/core/component.hpp"
#include "gxf/core/gxf.h"

namespace nvidia {
namespace gxf {

// This component holds data a single metric, where a metric is any double-typed value that is
// computed during the execution of a GXF application, and has success criteria defined by a
// lower threshold, an upper threshold, and an aggregation function that specifies how to
// aggregate multiple samples into a final value for the metric. Other components in a GXF app can
// access this component in order to, for example, write the metrics to a file for downstream
// analysis of app execution.
class Metric : public Component {
 public:
  gxf_result_t registerInterface(Registrar* registrar) override;
  gxf_result_t initialize() override;
  gxf_result_t deinitialize() override { return GXF_SUCCESS; }

  // Type for the aggregation functor.
  using aggregation_function_t = std::function<double(double)>;

  // Records a single sample to the metric and updates aggregated_value_.
  Expected<void> record(double sample);

  // Sets a custom function object to use when computing the aggregated_value_ over a set of
  // samples.
  Expected<void> setAggregationFunction(aggregation_function_t aggregation_function);

  // Checks whether the aggregated_value_ lies within the expected range.
  Expected<bool> evaluateSuccess();

  // Public accessor functions for metric data.
  Expected<double> getAggregatedValue();
  Expected<double> getLowerThreshold();
  Expected<double> getUpperThreshold();

  // Public functions to set a few common aggregation functions.
  // Computes the mean of a set of samples.
  Expected<void> setMeanAggregationFunction();
  // Computes the root-mean-square average over a set of samples.
  Expected<void> setRootMeanSquareAggregationFunction();
  // Finds the maximum absolute value over a set of samples.
  Expected<void> setAbsMaxAggregationFunction();
  // Finds the maximum over a set of samples.
  Expected<void> setMaxAggregationFunction();
  // Finds the minimum over a set of samples.
  Expected<void> setMinAggregationFunction();
  // Computes the total sum over a set of samples.
  Expected<void> setSumAggregationFunction();
  // Fixes the aggregated value to the last recorded sample.
  Expected<void> setFixedAggregationFunction();

 private:
  Parameter<std::string> aggregation_policy_;
  Parameter<double> lower_threshold_;
  Parameter<double> upper_threshold_;

  // Current aggregated value across all samples recorded to this metric. Updated on record().
  Expected<double> aggregated_value_ = Unexpected{GXF_UNINITIALIZED_VALUE};
  // Function object that is used to update the metric's aggregated value based on a given sample.
  // The function accepts a sample and returns the updated aggregated value. Called on record().
  aggregation_function_t aggregation_function_;
};

}  // namespace gxf
}  // namespace nvidia

#endif  // NVIDIA_GXF_STD_METRIC_HPP_
