// Copyright 2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright(C) 2024 eSOL Co., Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rmw_zenoh_pico/rmw_zenoh_pico.h>

#define ZenohPicoDataFunctions(T)			\
  T * T ## Generate(T *data)				\
  {							\
    if(data == NULL){					\
      data = (T *)Z_MALLOC(sizeof(T));			\
    }							\
    if(data != NULL){					\
      memset(data, 0, sizeof(T));			\
      z_mutex_init(&data->lock);			\
      data->ref += 1;					\
    }							\
    return data;					\
  }							\
  void T ## Destroy(T *data)				\
  {							\
    if(data->ref == 0) {				\
      z_mutex_drop(z_move(data->lock));			\
      Z_FREE(data);					\
    }							\
    return;						\
  }							\
  T * T ## RefClone(T *data)				\
  {							\
    z_mutex_lock(z_loan_mut(data->lock));		\
    data->ref += 1;					\
    z_mutex_unlock(z_loan_mut(data->lock));		\
    return data;					\
  }							\
  int T ## RefDec(T *data)				\
  {							\
    z_mutex_lock(z_loan_mut(data->lock));		\
    data->ref -= 1;					\
    z_mutex_unlock(z_loan_mut(data->lock));		\
    return data->ref;					\
  }							\

ZenohPicoDataFunctions(ZenohPicoEntity);
ZenohPicoDataFunctions(ZenohPicoTopicInfo);
ZenohPicoDataFunctions(ZenohPicoNodeInfo);

ZenohPicoDataFunctions(ZenohPicoSession);
ZenohPicoDataFunctions(ZenohPicoNodeData);
ZenohPicoDataFunctions(ZenohPicoSubData);
ZenohPicoDataFunctions(ZenohPicoPubData);
ZenohPicoDataFunctions(ZenohPicoServiceData);

ZenohPicoDataFunctions(ReceiveMessageData);

ZenohPicoDataFunctions(ZenohPicoTransportParams);
ZenohPicoDataFunctions(ZenohPicoWaitSetData);
