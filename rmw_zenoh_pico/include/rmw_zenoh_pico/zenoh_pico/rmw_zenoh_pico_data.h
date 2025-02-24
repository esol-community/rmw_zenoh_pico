/*
 * Copyright(C) 2024 eSOL Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rmw_zenoh_pico/rmw_zenoh_pico_init.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_subscription.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_publisher.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_service.h>
#include <rmw_zenoh_pico/rmw_zenoh_pico_rosMessage.h>

#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_entity.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_nodeInfo.h>
#include <rmw_zenoh_pico/zenoh_pico/rmw_zenoh_pico_topicInfo.h>

#ifndef RMW_ZENOH_PICO_DATA_H
#define RMW_ZENOH_PICO_DATA_H

#define ZenohPicoDataFunctionsEntry(T)		\
  extern T * T ## Generate(T *data);		\
  extern void T ## Destroy(T *data);		\
  extern T * T ## RefClone(T *data);		\
  extern int T ## RefDec(T *data);		\

ZenohPicoDataFunctionsEntry(ZenohPicoEntity);
ZenohPicoDataFunctionsEntry(ZenohPicoTopicInfo);
ZenohPicoDataFunctionsEntry(ZenohPicoNodeInfo);

ZenohPicoDataFunctionsEntry(ZenohPicoSession);
ZenohPicoDataFunctionsEntry(ZenohPicoNodeData);
ZenohPicoDataFunctionsEntry(ZenohPicoSubData);
ZenohPicoDataFunctionsEntry(ZenohPicoPubData);
ZenohPicoDataFunctionsEntry(ZenohPicoServiceData);

ZenohPicoDataFunctionsEntry(ReceiveMessageData);

ZenohPicoDataFunctionsEntry(ZenohPicoTransportParams);
ZenohPicoDataFunctionsEntry(ZenohPicoWaitSetData);

//
// rmw_zenoh_pico private data Generater utilities
//

#define ZenohPicoDataMutexLock(D)   z_mutex_lock(z_loan_mut((D)->lock))
#define ZenohPicoDataMutexUnLock(D) z_mutex_unlock(z_loan_mut((D)->lock))

#define ZenohPicoDataGenerate(D)					\
  _Generic((D),								\
	   ZenohPicoEntity *		: ZenohPicoEntityGenerate,	\
	   ZenohPicoTopicInfo *		: ZenohPicoTopicInfoGenerate,	\
	   ZenohPicoNodeInfo *		: ZenohPicoNodeInfoGenerate,	\
	   ZenohPicoSession *		: ZenohPicoSessionGenerate,	\
	   ZenohPicoNodeData *		: ZenohPicoNodeDataGenerate,	\
	   ZenohPicoSubData *		: ZenohPicoSubDataGenerate,	\
	   ZenohPicoPubData *		: ZenohPicoPubDataGenerate,	\
	   ZenohPicoServiceData *	: ZenohPicoServiceDataGenerate,	\
	   ReceiveMessageData *         : ReceiveMessageDataGenerate,	\
	   ZenohPicoTransportParams *	: ZenohPicoTransportParamsGenerate, \
	   ZenohPicoWaitSetData *	: ZenohPicoWaitSetDataGenerate	\
    )(D)

#define ZenohPicoDataDestroy(D)						\
  _Generic((D),								\
	   ZenohPicoEntity *		: ZenohPicoEntityDestroy,	\
	   ZenohPicoTopicInfo *		: ZenohPicoTopicInfoDestroy,	\
	   ZenohPicoNodeInfo *		: ZenohPicoNodeInfoDestroy,	\
	   ZenohPicoSession *		: ZenohPicoSessionDestroy,	\
	   ZenohPicoNodeData *		: ZenohPicoNodeDataDestroy,	\
	   ZenohPicoSubData *		: ZenohPicoSubDataDestroy,	\
	   ZenohPicoPubData *		: ZenohPicoPubDataDestroy,	\
	   ZenohPicoServiceData *	: ZenohPicoServiceDataDestroy,	\
	   ReceiveMessageData *         : ReceiveMessageDataDestroy,	\
	   ZenohPicoTransportParams *	: ZenohPicoTransportParamsDestroy, \
	   ZenohPicoWaitSetData *	: ZenohPicoWaitSetDataDestroy	\
    )(D)

#define ZenohPicoDataRefClone(D)					\
  _Generic((D),								\
	   ZenohPicoEntity *		: ZenohPicoEntityRefClone,	\
	   ZenohPicoTopicInfo *		: ZenohPicoTopicInfoRefClone,	\
	   ZenohPicoNodeInfo *		: ZenohPicoNodeInfoRefClone,	\
	   ZenohPicoSession *		: ZenohPicoSessionRefClone,	\
	   ZenohPicoNodeData *		: ZenohPicoNodeDataRefClone,	\
	   ZenohPicoSubData *		: ZenohPicoSubDataRefClone,	\
	   ZenohPicoPubData *		: ZenohPicoPubDataRefClone,	\
	   ZenohPicoServiceData *	: ZenohPicoServiceDataRefClone,	\
	   ReceiveMessageData *         : ReceiveMessageDataRefClone,	\
	   ZenohPicoTransportParams *	: ZenohPicoTransportParamsRefClone, \
	   ZenohPicoWaitSetData *	: ZenohPicoWaitSetDataRefClone	\
    )(D)

// #define ZenohPicoDataRelease(D)     (--(D)->ref == 0)
#define ZenohPicoDataRelease(D)						\
  (_Generic((D),							\
	    ZenohPicoEntity *		: ZenohPicoEntityRefDec,	\
	    ZenohPicoTopicInfo *	: ZenohPicoTopicInfoRefDec, \
	    ZenohPicoNodeInfo *		: ZenohPicoNodeInfoRefDec,	\
	    ZenohPicoSession *		: ZenohPicoSessionRefDec,	\
	    ZenohPicoNodeData *		: ZenohPicoNodeDataRefDec,	\
	    ZenohPicoSubData *		: ZenohPicoSubDataRefDec,	\
	    ZenohPicoPubData *		: ZenohPicoPubDataRefDec,	\
	    ZenohPicoServiceData *	: ZenohPicoServiceDataRefDec,	\
	    ReceiveMessageData *        : ReceiveMessageDataRefDec,	\
	    ZenohPicoTransportParams *	: ZenohPicoTransportParamsRefDec, \
	    ZenohPicoWaitSetData *	: ZenohPicoWaitSetDataRefDec	\
    )(D) == 0)

#endif
