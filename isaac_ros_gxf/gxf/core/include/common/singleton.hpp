// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#ifndef NVIDIA_COMMON_SINGLETON_HPP_
#define NVIDIA_COMMON_SINGLETON_HPP_

namespace gxf {

// A singleton which supports creation at pre-execution time
// Usage: For a class `MyType` with a member function `myMember` you can just write:
//   Singleton<MyType>::Get().myMember();
// This will automatically create a singleton, i.e. return the same instance each time you call Get.
// The constructor of MyType will be called at pre-execution time, i.e. before entering main.
// If you use multiple singletons they will all be created at pre-execution time, but you should
// not rely on a particular instantiation order.
template <typename T>
struct Singleton {
 public:
  Singleton& operator=(Singleton&) = delete;

  // Get the singleton
  static T& Get() {
    static T singleton;
    Use(dummy_);
    return singleton;
  }

 private:
  // Helpers to force pre-execution time
  static void Use(const T&) {}
  static T& dummy_;
};

// Force instantiation of the singleton at pre-execution time
template <typename T>
T& Singleton<T>::dummy_ = Singleton<T>::Get();

}  // namespace gxf

#endif  // NVIDIA_COMMON_SINGLETON_HPP_
