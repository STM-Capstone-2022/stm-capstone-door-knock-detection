/*
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Derived from simple_sub_pub_demo.c
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */


#include "logging_levels.h"
/* define LOG_LEVEL here if you want to modify the logging level from the default */

#define LOG_LEVEL    LOG_ERROR

#include "logging.h"


/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "kvstore.h"

#include "sys_evt.h"


/* MQTT library includes. */
#include "core_mqtt.h"

/* MQTT agent include. */
#include "core_mqtt_agent.h"
#include "mqtt_agent_task.h"

/* Subscription manager header include. */
#include "subscription_manager.h"

/* Sensor includes */
#include "b_u585i_iot02a_motion_sensors.h"

/**
 * @brief Size of statically allocated buffers for holding topic names and
 * payloads.
 */
#define MQTT_PUBLISH_MAX_LEN                 ( 200 )
#define MQTT_PUBLISH_PERIOD_MS               ( 500 )
#define MQTT_PUBLICH_TOPIC_STR_LEN           ( 256 )
#define MQTT_PUBLISH_BLOCK_TIME_MS           ( 200 )
#define MQTT_PUBLISH_NOTIFICATION_WAIT_MS    ( 1000 )
#define MQTT_NOTIFY_IDX                      ( 1 )
#define MQTT_PUBLISH_QOS                     ( MQTTQoS0 )

#define ISM330DHCX_INT1_Pin       GPIO_PIN_11
#define ISM330DHCX_INT1_GPIO_Port GPIOE
#define ISM330DHCX_INT1_EXTI_IRQn EXTI11_IRQn

static void DoubleTapDetected(void);
static void ISM330DHCX_Event_Handler(void * pvContext);
static void MX_GPIO_Init(void);

#define LONG_TIME 0xffff

/* Semaphore used to signal a double tap detection */
SemaphoreHandle_t xSemaphore = NULL;
/*-----------------------------------------------------------*/

/**
 * @brief Defines the structure to use as the command callback context in this
 * demo.
 */
struct MQTTAgentCommandContext
{
    MQTTStatus_t xReturnStatus;
    TaskHandle_t xTaskToNotify;
    uint32_t ulNotificationValue;
};

/*-----------------------------------------------------------*/
static void prvPublishCommandCallback( MQTTAgentCommandContext_t * pxCommandContext,
                                       MQTTAgentReturnInfo_t * pxReturnInfo )
{
    TaskHandle_t xTaskHandle = ( TaskHandle_t ) pxCommandContext;

    configASSERT( pxReturnInfo != NULL );

    uint32_t ulNotifyValue = pxReturnInfo->returnCode;

    if( xTaskHandle != NULL )
    {
        /* Send the context's ulNotificationValue as the notification value so
         * the receiving task can check the value it set in the context matches
         * the value it receives in the notification. */
        ( void ) xTaskNotifyIndexed( xTaskHandle,
                                     MQTT_NOTIFY_IDX,
                                     ulNotifyValue,
                                     eSetValueWithOverwrite );
    }
}

/*-----------------------------------------------------------*/

static BaseType_t prvPublishAndWaitForAck( MQTTAgentHandle_t xAgentHandle,
                                           const char * pcTopic,
                                           const void * pvPublishData,
                                           size_t xPublishDataLen )
{
    MQTTStatus_t xStatus;
    size_t uxTopicLen = 0;

    configASSERT( pcTopic != NULL );
    configASSERT( pvPublishData != NULL );
    configASSERT( xPublishDataLen > 0 );

    uxTopicLen = strnlen( pcTopic, UINT16_MAX );

    MQTTPublishInfo_t xPublishInfo =
    {
        .qos             = MQTT_PUBLISH_QOS,
        .retain          = 0,
        .dup             = 0,
        .pTopicName      = pcTopic,
        .topicNameLength = ( uint16_t ) uxTopicLen,
        .pPayload        = pvPublishData,
        .payloadLength   = xPublishDataLen
    };

    MQTTAgentCommandInfo_t xCommandParams =
    {
        .blockTimeMs                 = MQTT_PUBLISH_BLOCK_TIME_MS,
        .cmdCompleteCallback         = prvPublishCommandCallback,
        .pCmdCompleteCallbackContext = ( void * ) xTaskGetCurrentTaskHandle(),
    };

    if( xPublishInfo.qos > MQTTQoS0 )
    {
        xCommandParams.pCmdCompleteCallbackContext = ( void * ) xTaskGetCurrentTaskHandle();
    }

    /* Clear the notification index */
    xTaskNotifyStateClearIndexed( NULL, MQTT_NOTIFY_IDX );


    xStatus = MQTTAgent_Publish( xAgentHandle,
                                 &xPublishInfo,
                                 &xCommandParams );

    if( xStatus == MQTTSuccess )
    {
        uint32_t ulNotifyValue = 0;
        BaseType_t xResult = pdFALSE;

        xResult = xTaskNotifyWaitIndexed( MQTT_NOTIFY_IDX,
                                          0xFFFFFFFF,
                                          0xFFFFFFFF,
                                          &ulNotifyValue,
                                          pdMS_TO_TICKS( MQTT_PUBLISH_NOTIFICATION_WAIT_MS ) );

        if( xResult )
        {
            xStatus = ( MQTTStatus_t ) ulNotifyValue;

            if( xStatus != MQTTSuccess )
            {
                LogError( "MQTT Agent returned error code: %d during publish operation.",
                          xStatus );
                xResult = pdFALSE;
            }
        }
        else
        {
            LogError( "Timed out while waiting for publish ACK or Sent event. xTimeout = %d",
                      pdMS_TO_TICKS( MQTT_PUBLISH_NOTIFICATION_WAIT_MS ) );
            xResult = pdFALSE;
        }
    }
    else
    {
        LogError( "MQTTAgent_Publish returned error code: %d.", xStatus );
    }

    return( xStatus == MQTTSuccess );
}

/*-----------------------------------------------------------*/
static BaseType_t xInitSensors( void )
{
    int32_t lBspError = BSP_ERROR_NONE;

    /* Gyro + Accelerometer*/
    lBspError  = BSP_MOTION_SENSOR_Init  ( 0, MOTION_ACCELERO );
    lBspError |= BSP_MOTION_SENSOR_Enable( 0, MOTION_ACCELERO );

    return( lBspError == BSP_ERROR_NONE ? pdTRUE : pdFALSE );
}

/*-----------------------------------------------------------*/
void vMotionSensorsPublish( void * pvParameters )
{
    ( void ) pvParameters;
    BaseType_t xResult = pdFALSE;
    BaseType_t xExitFlag = pdFALSE;

    MQTTAgentHandle_t xAgentHandle = NULL;
    char pcPayloadBuf[ MQTT_PUBLISH_MAX_LEN ];
    char pcTopicString[ MQTT_PUBLICH_TOPIC_STR_LEN ] = { 0 };
    char * pcDeviceId = NULL;
    int lTopicLen = 0;

    xResult = xInitSensors();

    if( xResult != pdTRUE )
    {
        LogError( "Error while initializing motion sensors." );
        vTaskDelete( NULL );
    }

    /* We are using the semaphore for synchronisation so we create a binary
        semaphore rather than a mutex.  We must make sure that the interrupt
        does not attempt to use the semaphore before it is created! */
    xSemaphore = xSemaphoreCreateBinary();

    /* Enable interrupts on ISM330DHCX_INT1_Pin  */
    MX_GPIO_Init();

    pcDeviceId = KVStore_getStringHeap( CS_CORE_THING_NAME, NULL );

    if( pcDeviceId == NULL )
    {
        xExitFlag = pdTRUE;
    }
    else
    {
        lTopicLen = snprintf( pcTopicString, ( size_t ) MQTT_PUBLICH_TOPIC_STR_LEN, "%s/motion_sensor_data", pcDeviceId );
    }

    if( ( lTopicLen <= 0 ) || ( lTopicLen > MQTT_PUBLICH_TOPIC_STR_LEN ) )
    {
        LogError( "Error while constructing topic string." );
        xExitFlag = pdTRUE;
    }

    vSleepUntilMQTTAgentReady();

    xAgentHandle = xGetMqttAgentHandle();

    while( xExitFlag == pdFALSE )
    {
    	/* Block waiting for the semaphore to become available. */
    	if( xSemaphoreTake( xSemaphore, LONG_TIME ) == pdTRUE )
        {
            int lbytesWritten = snprintf( pcPayloadBuf, MQTT_PUBLISH_MAX_LEN, "{ \"double_tap\": true}");

            if( ( lbytesWritten < MQTT_PUBLISH_MAX_LEN ) && ( xIsMqttAgentConnected() == pdTRUE ) )
            {
                xResult = prvPublishAndWaitForAck( xAgentHandle, pcTopicString, pcPayloadBuf, ( size_t ) lbytesWritten );

                if( xResult != pdPASS )
                {
                    LogError( "Failed to publish motion sensor data" );
                }
            }
        }
    }

    vPortFree( pcDeviceId );
}

/**
  * @brief  Handle Double Tap detected event
  * @retval None
  */
static void DoubleTapDetected(void)
{
  static int32_t xHigherPriorityTaskWoken;

  /* Unblock the task by releasing the semaphore. */
  xSemaphoreGiveFromISR( xSemaphore, &xHigherPriorityTaskWoken );

  /* If xHigherPriorityTaskWoken was set to true you  we should yield.  The actual macro used here is port specific. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief  Handle ISM330DHCX Events
  * @retval None
  */
static void ISM330DHCX_Event_Handler(void * pvContext)
{
  BSP_MOTION_SENSOR_Status_t EventStatus = {0};
  int32_t lBspError = BSP_ERROR_NONE;

  pvContext = pvContext;

  /* Get event which generated the interrupt */
  lBspError = BSP_Get_Event_Status(0, MOTION_ACCELERO, &EventStatus);

  if(BSP_ERROR_NONE == lBspError)
  {
    if(EventStatus.DoubleTapStatus == 1)
    {
        DoubleTapDetected();
    }
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin : ISM330DHCX_INT1_Pin */
  GPIO_InitStruct.Pin  = ISM330DHCX_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ISM330DHCX_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ISM330DHCX_INT1_EXTI_IRQn, 5, 5);
  HAL_NVIC_EnableIRQ  (ISM330DHCX_INT1_EXTI_IRQn);

  GPIO_EXTI_Register_Callback(ISM330DHCX_INT1_Pin, ISM330DHCX_Event_Handler, NULL);
}
